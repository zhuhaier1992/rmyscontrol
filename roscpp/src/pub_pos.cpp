#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#ifndef RVO_SEED_RANDOM_NUMBER_GENERATOR
#define RVO_SEED_RANDOM_NUMBER_GENERATOR 1
#endif

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
#include "geometry_msgs/msg/pose2_d.hpp"

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

using namespace std::chrono_literals;
using namespace std;

class MinimalPublisher : public rclcpp::Node
{
    public:
        std::string s;
		ros2_interfaces::msg::Motions motions;
        MinimalPublisher() : Node("minimal_publisher"), count_(0), s("")
        {
            // std::string msssg;
            publisher_ = this->create_publisher<ros2_interfaces::msg::Motions>("rvo2", 10);
            timer_ = this->create_wall_timer(
                10ms, std::bind(&MinimalPublisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            // auto message = ros2_interfaces::msg::Motions();
            // message.data = "Hello, world! " + std::to_string(add(count_++, 100));
            // message.poses = this->s;
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            publisher_->publish(this->motions);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ros2_interfaces::msg::Motions>::SharedPtr publisher_;
        size_t count_;
        
};

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator *sim)
{
#if RVO_SEED_RANDOM_NUMBER_GENERATOR
	std::srand(static_cast<unsigned int>(std::time(NULL)));
#endif

	/* Specify the global time step of the simulation. */
	sim->setTimeStep(0.25f);

	/* Specify the default parameters for agents that are subsequently added. */
	sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);

	/*
	 * Add agents, specifying their start position, and store their goals on the
	 * opposite side of the environment.
	 */
	for (size_t i = 0; i < 5; ++i) {
		for (size_t j = 0; j < 5; ++j) {
			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f,  55.0f + j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, -75.0f));

			sim->addAgent(RVO::Vector2(55.0f + i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(-75.0f, 75.0f));

			sim->addAgent(RVO::Vector2(-55.0f - i * 10.0f, -55.0f - j * 10.0f));
			goals.push_back(RVO::Vector2(75.0f, 75.0f));
		}
	}

	/*
	 * Add (polygonal) obstacles, specifying their vertices in counterclockwise
	 * order.
	 */
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

	obstacle1.push_back(RVO::Vector2(-10.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 40.0f));
	obstacle1.push_back(RVO::Vector2(-40.0f, 10.0f));
	obstacle1.push_back(RVO::Vector2(-10.0f, 10.0f));

	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));

	sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	sim->addObstacle(obstacle3);
	sim->addObstacle(obstacle4);

	/* Process the obstacles so that they are accounted for in the simulation. */
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

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */
#ifdef _OPENMP
#pragma omp parallel for
#endif
	for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 0.1f) {
			goalVector = 0.1*RVO::normalize(goalVector);
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

bool reachedGoal(RVO::RVOSimulator *sim)
{
	/* Check if all agents have reached their goals. */
	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > 20.0f * 20.0f) {
			return false;
		}
	}

	return true;
}

// int main()
// {
// 	/* Create a new simulator instance. */
// 	RVO::RVOSimulator *sim = new RVO::RVOSimulator();

// 	/* Set up the scenario. */
// 	setupScenario(sim);

// 	/* Perform (and manipulate) the simulation. */
// 	do {
// #if RVO_OUTPUT_TIME_AND_POSITIONS
// 		updateVisualization(sim);
// #endif
// 		setPreferredVelocities(sim);
// 		sim->doStep();
// 	}
// 	while (!reachedGoal(sim));

// 	delete sim;

// 	return 0;
// }
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
    auto node = std::make_shared<MinimalPublisher>();
    	/* Create a new simulator instance. */
	RVO::RVOSimulator *sim = new RVO::RVOSimulator();

	/* Set up the scenario. */
	setupScenario(sim);

	/* Perform (and manipulate) the simulation. */
	do {
#if RVO_OUTPUT_TIME_AND_POSITIONS
		std::list<RVO::Vector2> li=updateVisualization(sim);
		node->s="";
		std::vector<geometry_msgs::msg::Pose2D> p2ds;
		for(RVO::Vector2 i:li){
			geometry_msgs::msg::Pose2D p2d;
			p2d.x=i.x();
			p2d.y=i.y();
			p2ds.push_back(p2d);
			node->s+=std::to_string(i.x())+","+std::to_string(i.y())+";";
		}
		// cout<<typeid(ptemp).name()<<endl;
		node->motions.poses=p2ds;
#endif
		setPreferredVelocities(sim);
		sim->doStep();
        rclcpp::spin_some(node);
		// auto message = std_msgs::msg::String();
		// // message.data = "Hello, world! " + std::to_string(add(count_++, 100));
		// message.data = node->s;
		// RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
		// node->publisher_->publish(message);
		// delay(1000);
	}	while (!reachedGoal(sim));

	delete sim;
    rclcpp::shutdown();
    return 0;
}