#include <cmath>
#include <cstdlib>

#include <vector>


#if RVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#if RVO_SEED_RANDOM_NUMBER_GENERATOR
#include <ctime>
#endif

#if _OPENMP
#include <omp.h>
#endif

#include <RVO.h>

#include <chrono>
#include <memory>
#include<typeinfo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_interfaces/msg/motions.hpp"
#include "ros2_interfaces/msg/ctrl.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <eigen3/Eigen/Dense>

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

using namespace std::chrono_literals;
using namespace std;
using namespace geometry_msgs::msg;
using std::placeholders::_1;
using namespace Eigen;
// using namespace Eigen::placeholders;
using namespace ros2_interfaces::msg;


static int num_rm=6; //number of robomasters
static int num_ys=6; //number of yanshees
static int num_p=1 + num_rm + num_ys; //plus 1 ball 
static RVO::RVOSimulator *sim = new RVO::RVOSimulator();
static bool working=false; //if received from node strategy

float limitPi(float a)
{
	if(a>=M_PI){
		a-=2*M_PI;
	}else if (a<-M_PI){
		a+=2*M_PI;
	}
	return a;
}

Vector3f pose2dToVector(Pose2D p)
{
	return Vector3f(p.x, p.y, p.theta);
}

Pose2D vectorToPose2d(Vector3f v)
{
	Pose2D p;
	p.x=v[0];
	p.y=v[1];
	p.theta=v[2];
	return p;
}

MatrixX3f pose2dsToMatrix(vector<Pose2D> poses)
{
	MatrixX3f res(num_p,3);
	for(int i = 0; i< num_p; ++i){
		// Vector3f v(poses[i].x, poses[i].y, poses[i].theta);
		res.row(i)=pose2dToVector(poses[i]);
	}
	return res;
}

void setCurrentPosition(RVO::RVOSimulator *sim, vector<Pose2D> p)
{
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 pv(p[i+1].x, p[i+1].y);
		sim->setAgentPosition(i, pv);
	}
}

Pose2D vector2ToPose2d(RVO::Vector2 v)
{
	Pose2D res;
	res.x=v.x();
	res.y=v.y();
	return res;
}

RVO::Vector2 pose2dToVector2(Pose2D p)
{
	RVO::Vector2 res(p.x, p.y);
	return res;
}

vector<RVO::Vector2> pose2dsToVector(vector<Pose2D> poses)
{
	vector<RVO::Vector2> res;
	for(Pose2D p : poses){
		res.push_back(pose2dToVector2(p));
	}
	return res;
}

Pose2D globalToRelative(RVO::Vector2 gp, Pose2D p, Pose2D goal)
{
	Vector2f gpv(gp.x(), gp.y());
	float t=p.theta;
	Matrix2f rr;
	rr<<cos(t), sin(t), -sin(t), cos(t);
	Vector3f rp=Vector3f::Zero();
	rp.head(2)=rr*gpv;
	rp[2]=limitPi(goal.theta-t);
	return vectorToPose2d(rp);
}

vector<Pose2D> getVelocity(RVO::RVOSimulator *sim, vector<Pose2D> p, vector<Pose2D> goals)
{
	vector<Pose2D> pose;
	pose.push_back(Pose2D());
	for(int i=0;i<num_rm;++i){
		RVO::Vector2 vi=sim->getAgentVelocity(i);
		// pose.push_back(vector2ToPose2d(vi));
		Pose2D rp=globalToRelative(vi, p[i+1], goals[i+1]);
		pose.push_back(rp);
	}
	return pose;
}

vector<Pose2D> initPoses(int len)
{
	return vector<Pose2D>(len, Pose2D());
}

class Rvo2 : public rclcpp::Node
{
    public:
        std::string s;
		Motions motions;
		Ctrl controls;
		vector<Pose2D> goals=initPoses(num_p); //length = num_p = 1*ball+num_rm+num_ys
		vector<Pose2D> p=initPoses(num_p); //same length
		vector<Pose2D> v=initPoses(num_p);
		rclcpp::Publisher<Ctrl>::SharedPtr publisher_;
        Rvo2() : Node("rvo2"), count_(0), s(""), 
			// p(MatrixX3f(num_p,3)), v(MatrixX3f(num_p,3)),
			global_u(MatrixX3f(num_rm,3)), goals(num_p,Pose2D())
        {
            publisher_ = this->create_publisher<Ctrl>("rvo2", 10);
            // timer_ = this->create_wall_timer(10ms, std::bind(&Rvo2::timer_callback, this));
			motions_sub_ = this->create_subscription<Motions>(          
      		"motions", 10, std::bind(&Rvo2::motions_callback, this, _1));
	  		strategy_sub_ = this->create_subscription<Ctrl>(          
      		"to_rvo", 10, std::bind(&Rvo2::rvo_callback, this, _1));
        }

    private:
        // void timer_callback()
        // {
        //     publisher_->publish(this->motions);
        // }
		void motions_callback(const Motions::SharedPtr msg)
		{
			p=msg->poses;
			v=msg->twists;
		}
		void rvo_callback(const Ctrl::SharedPtr msg)
		{
			if(working==false){
				cout<<"receiving strategy"<<endl;
				working=true;
			}
			controls.code=msg->code;
			copy_n(msg->pose.begin(), 1+num_rm, goals.begin());
			// for(Pose2D p : goals){
			// 	cout<<"x:"<<p.x<<"y:"<<p.y<<endl;
			// }
			// cout<<endl;
			
		}
		rclcpp::Subscription<Motions>::SharedPtr motions_sub_;
		rclcpp::Subscription<Ctrl>::SharedPtr strategy_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
		MatrixX3f global_u;
		
        size_t count_;
        
};

/* Store the goals of the agents. */
// std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator *sim, vector<Pose2D> p) 
{
	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.25f);

	sim->setAgentDefaults(5.0f, 10, 4.0f, 4.0f, 0.25f, 1.0f);
	for (int i =0; i<num_rm;++i){
		sim->addAgent(RVO::Vector2(p[i+1].x, p[i+1].y));
	}
	sim->setAgentDefaults(3.0f, 10, 4.0f, 4.0f, 0.10f, 0.01f);
	for (int i =num_rm; i<num_rm+num_ys;++i){
		sim->addAgent(RVO::Vector2(p[i+1].x, p[i+1].y));
	}

	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;
	float xmax=2.75, xmin = -2.82, ymax=2.4, ymin=-2.25, width=0.3;


	obstacle1.push_back(RVO::Vector2(xmax, ymin));
	obstacle1.push_back(RVO::Vector2(xmin, ymin));
	obstacle1.push_back(RVO::Vector2(xmin, ymin-width));
	obstacle1.push_back(RVO::Vector2(xmax, ymin-width));

	obstacle2.push_back(RVO::Vector2(xmax, ymin));
	obstacle2.push_back(RVO::Vector2(xmax+width, ymin));
	obstacle2.push_back(RVO::Vector2(xmax+width, ymax));
	obstacle2.push_back(RVO::Vector2(xmax, ymax));

	obstacle3.push_back(RVO::Vector2(xmax, ymax));
	obstacle3.push_back(RVO::Vector2(xmax, ymax+width));
	obstacle3.push_back(RVO::Vector2(xmin, ymax+width));
	obstacle3.push_back(RVO::Vector2(xmin, ymax));

	obstacle4.push_back(RVO::Vector2(xmin, ymin));
	obstacle4.push_back(RVO::Vector2(xmin, ymax));
	obstacle4.push_back(RVO::Vector2(xmin-width, ymax));
	obstacle4.push_back(RVO::Vector2(xmin-width, ymin));

	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	sim->addObstacle(obstacle3);
	sim->addObstacle(obstacle4);

	// /* Process the obstacles so that they are accounted for in the simulation. */
	sim->processObstacles();
}

#if RVO_OUTPUT_TIME_AND_POSITIONS
std::list<RVO::Vector2> updateVisualization(RVO::RVOSimulator *sim)
{
	std::list<RVO::Vector2> res;
	/* Output the current global time. */
	// std::cout << sim->getGlobalTime();

	/* Output the current position of all the agents. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        RVO::Vector2 posi=sim->getAgentPosition(i);
		// std::cout << " " << posi;
        // res+=std::to_string(posi.x())+ " "+std::to_string(posi.y())+" ";
		res.push_back(posi);
	}

	// std::cout << std::endl;
    return res;
}
#endif

void setPreferredVelocities(RVO::RVOSimulator *sim, vector<RVO::Vector2> goals)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i+1] - sim->getAgentPosition(i);
		// cout<<i<<": "<<goals[i].x()<<","<<goals[i].y()<<endl;
		if (RVO::absSq(goalVector) > 0.1f) {
			goalVector = RVO::normalize(goalVector);
		}
		
		sim->setAgentPrefVelocity(i, goalVector);

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
		                          dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

bool reachedGoal(RVO::RVOSimulator *sim, vector<RVO::Vector2> goals)
{
	/* Check if all agents have reached their goals. */
	cout<<"check reached"<<endl;
	cout<<goals.size()<<endl;
	cout<<sim->getAgentPosition(0)<<endl;
	for (size_t i = 0; i < num_rm; ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i+1]) > 20.0f * 20.0f) {
			cout<<"not reached"<<endl;
			return false;
		}
	}
	cout<<"reached"<<endl;
	return true;
}

void delay(int timeout_ms)
{
	auto start = std::chrono::system_clock::now();
	while (true)
	{
		auto duration =
			std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
		if (duration > timeout_ms){
		// LOGGING_ERROR("timeout occurred,timeout %d ms", timeout_ms);
			break;
		}
	}
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Rvo2>();
	rclcpp::spin_some(node);//如果有to_rvo则此时已更新node->controls，否则working==false	
	/* Set up the scenario. */
	setupScenario(sim, node->p);
	cout<<"inited"<<endl;
	/* Perform (and manipulate) the simulation. */
	do {
		setCurrentPosition(sim, node->p);
		// vector<RVO::Vector2> current_goal;
		if(working==false){
			node->goals=node->p;
		}
		setPreferredVelocities(sim, pose2dsToVector(node->goals));
		sim->doStep();//计算得到避障速度
		if(working){
			// vector<Pose2D> pose=getVelocity(sim);
			node->controls.pose=getVelocity(sim, node->p, node->goals);
			node->publisher_->publish(node->controls);
		}
		rclcpp::spin_some(node);
        
	}	while (rclcpp::ok());

	delete sim;
    rclcpp::shutdown();
    return 0;
}
// int main()
// {
// 	MatrixXd m(2,2), b(2,2), t(2,2), s(2,2);
// 	Matrix4d a;
// 	int x=10;
// 	cout<<"num_p:"<<num_p;
// 	MatrixX3f xx(num_p, 3);
// 	a.setZero();
// 	m<<1,2,3,4;
// 	a.col(0)=m.reshaped<RowMajor>().transpose();
// 	b<<5,6,7,8;
// 	t=(m.array()*b.array()).matrix()*m;
// 	s<<5,12,21,32;
// 	s=s*m;
// 	cout<<"m:\n"<<xx<<endl;
// 	cout<<s(all, last)<<endl;
// 	cout<<"a:\n"<<a<<endl;
// }