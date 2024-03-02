# updated: 2024.01.27 
import rclpy
import random
import time
import math
from mods import YanAPI

from rclpy.node import Node
from ros2_interfaces.msg import Motions,YansheeKicked
from geometry_msgs.msg import Twist, Vector3, Pose2D
from math import pi

_YANSHEE_IDLE = 0
_YANSHEE_WANDER = 1
_YANSHEE_BALLCATCH = 2

_BALL_HAS_INTERCEPT = 0
_BALL_NO_INTERCEPT = 1

_SEG_UPPER = 1000
_SEG_LEFT_UP = -110
_SEG_LEFT_DOWN = -120
_SEG_LOWER = -1000
_SEG_RIGHT_UP = -210
_SEG_RIGHT_DOWN = -220
_SEG_YANSHEE_1 = 110
_SEG_YANSHEE_2 = 120
_SEG_YANSHEE_3 = 130
_SEG_YANSHEE_4 = 210
_SEG_YANSHEE_5 = 220
_SEG_YANSHEE_6 = 230

# time utils
_START_TIME = time.time()

class YansheePose():
    def __init__(self,x=0.,y=0.,theta=0.,vx=0.,vy=0.):
        self.x, self.y,self.vx,self.vy,self.theta = x,y,vx,vy,theta
        self.timestamp = time.time()
    
    def setTime(self):
        self.timestamp = time.time()

class YansheeParameters(YansheePose):
    def __init__(self,ip:str,id:int,name:str,anchor:Pose2D,side:str ='left',responsible:tuple=(-1.2,-0.7)):
        super(YansheeParameters,self).__init__()

        self.ip,self.id,self.name,self.anchor,self.side,self.responsible = ip,id,name,anchor,side,responsible

        self.state = _YANSHEE_IDLE
    
    def setPose(self,pose:Pose2D):
        self.x,self.y,self.theta = pose.x,pose.y,pose.theta
    
    def setSpeed(self,spd:Pose2D):
        self.vx,self.vy = spd.x,spd.y
    
    def setMotion(self,Pose:Pose2D,Speed:Pose2D):
        self.setPose(pose=Pose)
        self.setSpeed(spd=Speed)
        self.setTime()
    
    def setResponsible(self,lower_y:float,upper_y:float):
        self.responsible = (lower_y,upper_y)
    
    def isResponsibleY(self,aimedY:float): # judge if the given Y is inside its responsible area
        return (self.responsible[0] <= aimedY) and (self.responsible[1]>=aimedY)
    
    def printDeviation(self,target:tuple)->float: # publish target's relative pos with yanshee
        _dx,_dy = target[0]-self.x,target[1]-self.y
        print("Target Relative Pos:%.2f,%.2f"%(_dx,_dy))
        return _dx,_dy

    def printMotion(self):
        print("Yanshee %d: x=%.2f,y=%.2f,theta=%.2f deg,vx=%.2f,vy=%.2f"%(self.id-6,self.x,self.y,self.theta,self.vx,self.vy))

    def printInfo(self):
        print("Yanshee %d: id=%d,ip=%s,name=%s,anchor:(%.2f,%.2f,%.2f),side:%s"%(
            self.id-6,self.id,self.ip,self.name,self.anchor.x,self.anchor.y,self.anchor.theta,self.side
        ))

class BallEstimator(Node):
    def __init__(self,name='ball_est',verbose=True):
        super(BallEstimator,self).__init__(name)

        self.name = name
        self.verbose = verbose

        self.x = -0.114
        self.y = -0.514


        self.vx = 1.14
        self.vy = 0.514

        self.timestamp = time.time()

        self.last_timestamp = time.time()

        self.log = self.get_logger()

        # field border coords
        self.upperY = 1.2
        self.lowerY = -1.2
        self.leftX = -2.5
        self.rightX = 2.7
        # Field size
        self.dimY = self.upperY - self.lowerY #2 meters along Y axis: 1.2 + 1.2
        self.dimX = self.rightX - self.leftX#5 meters along X axis: 2.5 + 2.7        

        # four field border's (A,B,C) linear function
        self.upperLineABC = (0,1,-self.upperY) # y = upperY
        self.lowerLineABC = (0,1,-self.lowerY) # y = lowerY
        self.leftLineABC = (1,0,-self.leftX) # x = leftx
        self.rightLineABC = (1,0,-self.rightX) # x= rightx

        # static params: Yanshees
        # anchor_left y: -0.7, -0.35,-1.0
        # anchor_right y: -1.0, -0.35, 0.7

        self.anchors = (-1.0,-0.35,0.7,-1.0,-0.35,0.7)

        self.yanshee_1 = YansheeParameters(ip='192.168.3.51',name='6AB8@1',id=7,anchor=Pose2D(x=-2.5,y=-1.0,theta=0.),side='left')
        self.yanshee_2 = YansheeParameters(ip='192.168.3.56',name='A77D@2',id=8,anchor=Pose2D(x=-2.5,y=-0.35,theta=0.),side='left')
        self.yanshee_3 = YansheeParameters(ip='192.168.3.76',name='DF7D@3',id=9,anchor=Pose2D(x=-2.5,y=0.7,theta=0.),side='left')
        self.yanshee_4 = YansheeParameters(ip='192.168.3.50',name='E6BD@4',id=10,anchor=Pose2D(x=2.7,y=-1.0,theta=pi),side='right')
        self.yanshee_5 = YansheeParameters(ip='192.168.3.15',name='A61C@5',id=11,anchor=Pose2D(x=2.7,y=-0.35,theta=pi),side='right')
        self.yanshee_6 = YansheeParameters(ip='192.168.3.18',name='6280@6',id=12,anchor=Pose2D(x=2.7,y=0.7,theta=pi),side='right')

        # Yanshee responsible areas:
  
        # left,right segmenting points that devide two neighbored yanshee responsible areas
        # View this as a walkaround of the Y axis: from lower_Y to upper_Y
        # lower Y: -1.20
        LSEG1, RSEG1 = -1.15, -1.15
        # anchor 1: -1.00
        LSEG2, RSEG2 = -0.67, -0.67
        # anchor 2: -0.35
        LSEG3, RSEG3 = 0.18, 0.18
        # anchor 3: +0.70
        LSEG4, RSEG4 = 0.90, 0.90
        # upper Y: +1.20
        self.leftSegPoints = (LSEG1,LSEG2,LSEG3,LSEG4)
        self.rightSegPoints = (RSEG1,RSEG2,RSEG3,RSEG4) 

        # set Yanshee responsible areas
        self.yanshee_1.setResponsible(LSEG1,LSEG2)
        self.yanshee_2.setResponsible(LSEG2,LSEG3)
        self.yanshee_3.setResponsible(LSEG3,LSEG4)
        self.yanshee_4.setResponsible(RSEG1,RSEG2)
        self.yanshee_5.setResponsible(RSEG2,RSEG3)
        self.yanshee_6.setResponsible(RSEG3,RSEG4)

        # Yanshee list
        self.yanshees = [self.yanshee_1,self.yanshee_2,self.yanshee_3,self.yanshee_4,self.yanshee_5,self.yanshee_6]
        
        # rbtmaster list # TODO: not used now
        self.rbtmasters = [] # not used

        # math parameters # 114514 = not inited
        # y = kx + b
        self.tangent = 114.514 # slope. 
        self.bias = 11.4514
        # ball's moving line's params A,B,C as for Ax+By+C=0
        # Ax + By + C=0
        self.lABC = (11,41,51)
        # ball's moving line's crossPoints with four borders
        self.leftCrossXY = (self.leftX,114514)
        self.rightCrossXY = (self.rightX,114514)
        self.upperCrossXY = (114514,self.upperY)
        self.lowerCrossXY = (114514,self.lowerY)

        #-- ball aiming point calc by getAim
        self.ballAimedBorder = 'Not initialized' # enum('left','right','upper','lower')
        self.ballAimedPoint = (0.114,0.514) #'not initialized'
        self.moveTo = "Not initialized"
        #-- ball requires which yanshee to move
        self.ballHoldYansheeID = 0 # int8. The final msg to publish. enum(0,1,2,3,4,5,6); 0 = not req any yanshee, -1 = errors found
        self.ballAimedSegment = "_SEG_NOT_INITED" # string in format _SEG_XXXXX or _SEG_YANSHEE_X
        #-- volatile params updatable
        self.msg = None # pos msg retrieved from Motions_b
        self.msg_sent = None # published msg
        self.poses = [] # parsed pos msg: Poses
        self.twists = [] # parsed speed msg: Speeds

        #-- Motions subscriber
        self.pos_subscriber = self.create_subscription(msg_type=Motions,topic='motions_b',callback=self.getPoseFromROS,qos_profile=10)
        #-- Yanshee ID publisher
        self.yansheeIDPublisher = self.create_publisher(msg_type=YansheeKicked,topic='yanshee_id',qos_profile=10)
        #-- Yanshee ID publish timer
        self.publish_timer = self.create_timer(timer_period_sec=0.5,callback=self.timerPublishCallback)

        self.publish_cnt = 0
        self.pos_subs_cnt = 0

        # print yanshee info
        for _ys in self.yanshees:
            _ys.printInfo()
        print('Initialized')
    
    def getBallXY(self) -> tuple: # return (x,y)
        return (self.x,self.y)
    
    def getBallVxVy(self) -> tuple: #return (vx,vy)
        return (self.vx,self.vy)
    
    def printBall(self):
        print("Ball: x=%.2f,y=%.2f,vx=%.2f,vy=%.2f"%(self.x,self.y,self.vx,self.vy))
 
    def printMsgContent(self): # print msg received from ros and sent to ros
        _msg = self.msg
        if _msg == None or _msg == []:  
            self.log.warning('Empty Msg Received from ROS:'+str(_msg))
        else:
            _poses = _msg.poses
            _twists = _msg.twists

            _g = timeElapsed()
            print("TIME=%.2f,%dth run of .getPoseFromROS()::"%(_g,self.pos_subs_cnt))
            print("msg length: %d poses, %dtwists"%(len(_poses),len(_twists)))
   
            print("Poses:")
            for i,_p in enumerate(_poses):
                print("ID=%d,X=%.2f,Y=%.2f,Facing %.2f or %.1fdeg"%(i+1,_p.x,_p.y,_p.theta,math.degrees(_p.theta)))

            print("Twists")
            for i,_t in enumerate(_twists):
                print("ID=%d,Vx=%.2f,Vy=%.2f,Direction:%.2f or %.1f deg"%(i+1,_t.x,_t.y,_t.theta,math.degrees(_t.theta)))
        # also show the msg it pubs
        _mst = self.msg_sent
        print("Message sent to ROS: Yanshee ID=",_mst)

    def printYanshees(self): # print all yanshees' motions
        for i in range(6):
            self.yanshees[i].printMotion()

    def printMathDetails(self): #print math calculations
        print("Math speculations of %dth turn at time=%.2f"%(self.publish_cnt,timeElapsed()))
        print("Current Ball Pos")
        self.printBall()
        # print linear equations and interceptions
        print('Ball line function: %.2fx+%.2fy+%.2f=0'%self.lABC)
        print('Ball line function states in y=kx+b: y=%.2fx+%.2f'%(self.tangent,self.bias))
        print("Ball line's interceptions with four borders:")
        print('Upper Bound where y=%.2f:: %.2f,%.2f'%(self.upperY,self.upperCrossXY[0],self.upperCrossXY[1]))
        print('Lower Bound where y=%.2f:: %.2f,%.2f'%(self.lowerY,self.lowerCrossXY[0],self.lowerCrossXY[1]))
        print('Left Bound where x=%.2f:: %.2f,%.2f'%(self.leftX,self.leftCrossXY[0],self.leftCrossXY[1]))
        print('Left Bound where x=%.2f:: %.2f,%.2f'%(self.rightX,self.rightCrossXY[0],self.rightCrossXY[1]))
        print("Ball is heading towards: %s,aiming towards border: %s"%(self.moveTo,self.ballAimedBorder))
        print("Ball will first intercept with: %s at "%self.ballAimedSegment.lstrip('_SEG_'),self.ballAimedPoint)
        _id = self.ballHoldYansheeID
        
        # print which Yanshee to move
        if _id > 0:
            _yan = self.yanshees[_id-1]
            _yl,_yu = _yan.responsible
            if _id <= 3:_x = self.leftX
            else:_x = self.rightX
            print("Responsible Yanshee: %d,responsible range: x=%.2f,y: %.2f -- %.2f"%(_id,_x,_yl,_yu))
        else:
            print("No Yanshee is Required to Catch Ball!")
            return
        
        # print Responsible Yanshee's Data
        print("Responsible Yanshee's Info:")
        _yan.printInfo()
        print("Responsible Yanshee's Motion:")
        _yan.printMotion()
        print("Yanshee's Destination:",self.ballAimedPoint)
        _yan.printDeviation(target=self.ballAimedPoint)

        print("End of Maths")
        return
        
    def getPoseFromROS(self,msg=[],verbose=False) -> bool: # pos_retrieval subscriber's callback func
        self.msg = msg
        self.timestamp = time.time()
        if msg == None or msg == []:  
            self.log.warning('Empty Msg:',msg)
            return False
        else:
            poses = msg.poses
            twists = msg.twists
            # also update self's raw data storage
            self.poses = poses
            self.twists = twists
            self.pos_subs_cnt +=1
            if verbose:
                self.printMsgContent()
            # ball
            _pos = poses[0]
            _tws = twists[0]

            self.last_x,self.last_y,self.last_vx,self.last_vy, self.last_timestamp = self.x,self.y,self.vx,self.vy,self.timestamp

            self.x, self.y = _pos.x, _pos.y
            self.vx, self.vy = _tws.x, _tws.y
            self.timestamp = time.time()

            # Yanshees
            for id in range(7,13):
                _pos = poses[id]
                _tws = twists[id]

                self.yanshees[id-7].setMotion(Pose=_pos,Speed=_tws)
                if verbose:
                    print('Yanshee motions:')
                    self.yanshees[id-7].printMotion()
            if verbose:
                self.printBall()
            return True
    
    def _ballK(self)->float: # calc ball orient from vx vy into y=kx+b
        try: 
            # k
            self.tangent = self.vy/self.vx
            # b
            self.bias = self.y - self.tangent * self.x 
        except ZeroDivisionError:
            if self.vy > 0:
                self.tangent = pi/2
            elif self.vy < 0:
                self.tangent = -pi/2
            else:
                self.log.warn('self.ballSpeedTangent(): warn:ball vx and vy both are 0.0, cannot infer speed angle. assume it is 0.114514 instead of math.nan')
                # self.tangent = math.nan
                self.tangent = 0.114514
                self.bias = 0.114514
        return self.tangent # self.bias not needed to return

    def _ballABC(self)->tuple: # give ball speedline function params (A,B,C) such that A*x + B*y + C=0

        # A=-K, B=1,C= KX0 - Y0
        k = self._ballK()
        A = -k
        B = 1
        C = k*self.x - self.y
        self.la,self.lb,self.lc =A,B,C
        self.lABC = (A,B,C) 
        return (A,B,C)
    
    def _getBallAimedPoint(self)->tuple: # calculate ball aiming segment and aimed point on the field border
        # get ball's line's function Ax+By+C=0
        lABC = self._ballABC()
        # get ball's line's four crosses' coords (X,Y) with field borders
        self.leftCrossXY = crossPoint(lABC,self.leftLineABC)
        self.rightCrossXY = crossPoint(lABC,self.rightLineABC)
        self.upperCrossXY = crossPoint(lABC,self.upperLineABC)
        self.lowerCrossXY = crossPoint(lABC,self.lowerLineABC)

        # judge which directions are ball moving to
        if self.vx > 0.0 and self.vy>0.0:
            self.moveTo = 'right_upper'
            if self.upperCrossXY[0] < self.rightCrossXY[0]:
                # ball coming towards upper bound of field. out
                self.ballAimedBorder = 'upper'
                self.ballAimedPoint = self.upperCrossXY
            elif self.upperCrossXY[1] >= self.rightCrossXY[1]:
                self.ballAimedBorder = 'right'
                self.ballAimedPoint = self.rightCrossXY
            else:
                print('warn: _getBallAimedPoint():: right_upper:  ball cross data not coherent')

        elif self.vx >0.0 and self.vy <0.0:
            self.moveTo = 'right_lower'
            if self.lowerCrossXY[0] < self.rightCrossXY[0]:
                self.ballAimedBorder = 'lower'
                self.ballAimedPoint = self.lowerCrossXY
            elif self.lowerCrossXY[1] < self.rightCrossXY[1] :
                self.ballAimedBorder = 'right'
                self.ballAimedPoint =self.rightCrossXY
            else:
                print('warn: _getBallAimedPoint():: right_lower:  ball cross data not coherent')
        elif self.vx <0.0 and self.vy >0.0:
            self.moveTo = 'left_upper'
            if self.upperCrossXY[0] > self.leftCrossXY[0]:
                self.ballAimedBorder = 'upper'
                self.ballAimedPoint = self.upperCrossXY
            elif self.leftCrossXY[1] < self.upperCrossXY[1]:
                self.ballAimedBorder = 'left'
                self.ballAimedPoint = self.leftCrossXY
            else:
                print('warn: _getBallAimedPoint():: left_upper:  ball cross data not coherent')
                
        elif self.vx <0.0 and self.vy <0.0:
            self.moveTo = 'left_lower'
            if self.lowerCrossXY[0] > self.leftCrossXY[0]:
                self.ballAimedBorder = 'lower'
                self.ballAimedPoint = self.lowerCrossXY
            elif self.lowerCrossXY[1] < self.leftCrossXY[1]:
                self.ballAimedBorder = 'left'
                self.ballAimedPoint = self.leftCrossXY
            else:
                print('warn: _getBallAimedPoint():: left_lower:  ball cross data not coherent')
        else:
            print('warn:_getBallAimedPoint(): ball speed (0.0,0.0), cannot determine ball aim')

        return self.ballAimedBorder,self.ballAimedPoint
            
    def getBallAimedYanshee(self)->int: # calc which yanshee should be responsible for the ball
        #using self.data,assuming all params are latest
        _seg,_aim = self._getBallAimedPoint()
        if _seg == 'upper':
            self.ballAimedSegment = "_SEG_UPPER"
            self.ballHoldYansheeID = 0
        elif _seg == 'lower':
            self.ballAimedSegment = "_SEG_LOWER"
            self.ballHoldYansheeID = 0
        elif _seg == 'left':
            _y = _aim[1]
            if _y < self.upperY and _y >= self. leftSegPoints[3]:
                self.ballAimedSegment = "_SEG_LEFT_UP"
                self.ballHoldYansheeID = 0
            elif _y < self.leftSegPoints[3] and _y >= self.leftSegPoints[2]:
                self.ballAimedSegment = "_SEG_YANSHEE_3"
                self.ballHoldYansheeID = 3
            elif _y < self.leftSegPoints[2] and _y >= self.leftSegPoints[1]:
                self.ballAimedSegment = "_SEG_YANSHEE_2"
                self.ballHoldYansheeID = 2
            elif _y < self.leftSegPoints[1] and _y >=self.leftSegPoints[0]:
                self.ballAimedSegment = "_SEG_YANSHEE_1"
                self.ballHoldYansheeID = 1
            elif _y < self.leftSegPoints[0] and _y >= self.lowerY:
                self.ballAimedSegment = "_SEG_LEFT_DOWN"
                self.ballHoldYansheeID = 0
            else:
                self.ballHoldYansheeID = -1
                print('error: getBallAimedSegment(): _seg = left: data incoherent')
        elif _seg == 'right':
            _y = _aim[1]
            if _y < self.upperY and _y >= self. rightSegPoints[3]:
                self.ballAimedSegment = "_SEG_RIGHT_UP"
                self.ballHoldYansheeID = 0
            elif _y < self.rightSegPoints[3] and _y >= self.rightSegPoints[2]:
                self.ballAimedSegment = "_SEG_YANSHEE_6"
                self.ballHoldYansheeID = 6
            elif _y < self.rightSegPoints[2] and _y >= self.rightSegPoints[1]:
                self.ballAimedSegment = "_SEG_YANSHEE_5"
                self.ballHoldYansheeID = 5
            elif _y < self.rightSegPoints[1] and _y >=self.rightSegPoints[0]:
                self.ballAimedSegment = "_SEG_YANSHEE_4"
                self.ballHoldYansheeID = 4
            elif _y < self.rightSegPoints[0] and _y >= self.lowerY:
                self.ballAimedSegment = "_SEG_RIGHT_DOWN"
                self.ballHoldYansheeID = 0
            else:
                self.ballHoldYansheeID = -1
                print('error: getBallAimedSegment(): _seg = right: data incoherent')
        else:
            self.ballHoldYansheeID = -1
            print('error: getBallAimedSegment():_seg value error')
        return self.ballHoldYansheeID

    def printAll(self): #print Msg received and sent, Yanshee motions, Ball Motion and How they are calculated in math details
        print("printAll info at %.2f second"%timeElapsed())
        print('=='*40)
        self.printMsgContent()
        print("--"*30)
        self.printYanshees()
        print("--"*30)        
        self.printBall()
        print("--"*30)
        self.printMathDetails()
    
    def timerPublishCallback(self): # Yanshee publish timer's callback func

        # run 
        self.getBallAimedYanshee()

        # msg to publish
        _s = YansheeKicked()

        # load msg content
        _s.id = self.ballHoldYansheeID
        self.msg_sent = _s

        # publish msg
        self.yansheeIDPublisher.publish(_s) # int8; YansheeKicked.msg/id

        self.publish_cnt += 1
        # msg publish notifier
        _tm = timeElapsed()
        _ts = '%.2f'%(_tm)
        print('TIME=%s:: BallEstimator: %dth publish: ID=%d'%(_ts,self.publish_cnt,_s.id))
        if self.verbose:
            self.printAll()

def crossPoint(line1:tuple,line2:tuple)->tuple: # given (A1,B1,C1) (A2,B2,C2), return Intercept Point: tuple(float,float)
    A1,B1,C1=line1
    A2,B2,C2=line2

    _TOL = 1e-4

    # parallel lines
    if abs(A1*B2-A2*B1) < _TOL:
        print('crossPoint(): warn: parallel lines {} and {} has no intercept. return (10000,10000)'.format(line1,line2))
        return (10000.0,10000.0)
    else:
        resd = A1*B2 - A2*B1
        xres = -C1*B2 + C2*B1
        yres = -A1*C2 + A2*C1

        x = xres / resd
        y = yres / resd
        return (x,y)

def timeElapsed(): # show how many secs elapsed since the first launch of the script
    return time.time() - _START_TIME

def main(*args,**kwargs):
    rclpy.init()
    print("Yanshee ID service start at %.3f"%time.time())
    game = BallEstimator(name='Choose_the_Closet_Yanshee_to_Catch_Upcoming_Ball')
    sp_cnt = 0
    err_cnt = 0
    _MAX_ERR_ALLOWED = 10
    _MAX_TURNS_SPIN = 1000
    while(sp_cnt<_MAX_TURNS_SPIN):
        try:
            sp_cnt+=1
            _st = time.time()
            print("**"*50)
            game.log.warn("No.%dth spin at %.2f second:"%(sp_cnt,timeElapsed()))

            rclpy.spin_once(node=game)

            _se = time.time()

            print("%dth Spin Finished in %.2f Seconds"%(sp_cnt,_se-_st))

            _SLEEP_TIME = 0
            if _SLEEP_TIME > 0:
                time.sleep(_SLEEP_TIME)
                print("Slept %.2f seconds..."%(_SLEEP_TIME))
                _ss = time.time()
                print("%dth Spin Used %.2f Seconds in Total"%(sp_cnt,_ss-_st))
        except Exception as e:
            err_cnt +=1
            _err = "An Exception Occurred at %dth Spin at %.2f sec for %dth time:\n"% (sp_cnt,timeElapsed(),err_cnt) 
            game.get_logger().fatal(_err + str(e))
            if err_cnt >= _MAX_ERR_ALLOWED:
                game.get_logger().fatal("%d Errors happened in total, Program Exits Abnormaly!"%err_cnt)
                raise e

    print("ALL %d Spins Finished in %.2f seconds, average %.2f sec for 1 turn! Program Exits Normally."%(sp_cnt,timeElapsed(),timeElapsed()/sp_cnt))
    game.destroy_node()
            
if __name__ == '__main__':
    main()
            






