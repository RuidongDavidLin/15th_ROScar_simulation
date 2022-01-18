#!/usr/bin/env python  
# coding=UTF-8
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist #,PoseStamped  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt, sin, cos
import geometry_msgs.msg


from start_point_trans import start_point_trans
from pathPlanning import pathPlanning



class MultiNav():  
    def __init__(self):  
        rospy.init_node('MultiNav', anonymous=False)  
        rospy.on_shutdown(self.shutdown)  
        pub=rospy.Publisher('/move_base_simple_goal',geometry_msgs.msg.PoseStamped,queue_size=10)
        # How long in seconds should the robot pause at each location?  
        self.rest_time = rospy.get_param("~rest_time", 10)  
        
        


        '''
        angle transform
        '''
        rospy.loginfo("Click on the map in RViz to set the intial pose...")  
        #self.last_location = Pose()  
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)    
        while(1):
            rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)  
            
            rospy.loginfo(orientation)

            
            print ("start:")
            startL=int(input())
            
        #param for translate angle  
            trans=start_point_trans(0)
            start_point_transed = trans.fromOriginalToMarked(startL,orientation)
            rospy.loginfo(start_point_transed)
            
            if start_point_transed != 0:
                break
        
        print ("mast point num:")
        mPN=int(input())
        mustPointL=[]
        for i in range(mPN):
            print ('must point num:',i+1)
            mustPointL.append(int(input()))
        print ('end:')
        endL=int(input())
        print (startL,mustPointL,endL)
        
        '''
        /////////////////////////////////////////////////////////////////////////////
        '''

        stpoint=start_point_transed
        demo=pathPlanning(1,0,90,1)
        path=demo.findBestPathToCoordinateOnlyTarget(stpoint,mustPointL,endL)
        #path=[[3.0,0],[9,0],[12,0],[12,5]]
        rospy.loginfo(path)

        '''
        ///////////////////////////////////////////////////////////////
        Load Waypoints From PathPlaning
        '''
        waypoints = list() 
        #send waypoint(trajectory planned) to list "waypoint"
        for goal in path:
            #coordinate transform (origin to rviz)
            
            '''
            theta=90 #rotation angle 
            a=1     #offset x
            b=1     #offset y

            goal_x=float(goal[0])*cos(theta)+float(goal[1])*sin(theta)+a
            goal_y=float(goal[1])*cos(theta)-float(goal[0])*sin(theta)+b
            '''
            
            goal_x=float(goal[0])
            goal_y=float(goal[1])
            
            waypoints.append(Pose(Point(goal_x, goal_y, 0.0), Quaternion(0.0, 0.0, 1.0, 0.0)))
        '''
        ////////////////////////////////////////////////////////////////
        '''
        

        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  
        rospy.loginfo("Waiting for move_base action server...")  
  
        # Wait 60 seconds for the action server to become available  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
          
        # A variable to hold the initial pose of the robot to be set by the user in RViz  
        #initial_pose = PoseWithCovarianceStamped()  
        # Variables to keep track of success rate, running time, and distance traveled  
        




        start_time = rospy.Time.now()    
        rospy.loginfo("Starting navigation")  
    

        '''
        ////////////////////////////////////////////////////////////////////////
        Send Mutiple Goals To Rcaecar
        '''
        i=0 #counter "i"
        num_goals = len(waypoints)

        # Set up the goal location  (/move_base_simple/goal)
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.pose = waypoints[i]
        rospy.loginfo(self.goal.target_pose.pose)
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.header.stamp = rospy.Time.now() 
    # #Test to publish geometry_msgs
        msg=geometry_msgs.msg.PoseStamped()  
        msg.header.frame_id='map'
        msg.header.stamp=rospy.Time.now()
        msg.pose= waypoints[i]
        self.move_base.send_goal(self.goal, done_cb=None, active_cb=active_cb, feedback_cb=self.feedback_cb)
        pub.publish(msg)
        rospy.loginfo("Going to: " + str(i)) 
        
        dist=self.distances(current_point)
                   
        
        # Start the robot toward the next location      
        # Begin the main loop and run through a sequence of list waypoints  
        while i < num_goals and not rospy.is_shutdown():  
            dist=self.distances(current_point)
            
            if dist < 2:
                

            # Set up the next goal location  (/move_base_simple/goal)
                self.goal = MoveBaseGoal()  
                self.goal.target_pose.pose = waypoints[i]  
                self.goal.target_pose.header.frame_id = 'map'  
                self.goal.target_pose.header.stamp = rospy.Time.now()  
            # Let the user know where the robot is going next  
                rospy.loginfo("Going to: " + str(i)) 
            # #set up /move_base_simple/goal
                msg=geometry_msgs.msg.PoseStamped()
                msg.header.frame_id='map'
                msg.header.stamp=rospy.Time.now()
                msg.pose= waypoints[i]
                self.move_base.send_goal(self.goal, done_cb=None, active_cb=active_cb, feedback_cb=self.feedback_cb)
                pub.publish(msg)
            # #end send
                
                i += 1  
                                      
            # Start the robot toward the next location  

            '''
            ////////////////////////////////////////////////////////////////////
            '''
                
  
    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose 
        global orientation
        orientation=initial_pose.pose.pose.orientation.z
         

    def distances(self,current_point):
        dis=sqrt(pow(current_point.x-self.goal.target_pose.pose.position.x,2)+pow(current_point.y-self.goal.target_pose.pose.position.y,2))
        #rospy.loginfo(dis)
        return dis
    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)
    def feedback_cb(self,msg):
        current_point.x=msg.base_position.pose.position.x
        current_point.y=msg.base_position.pose.position.y
        current_point.z=msg.base_position.pose.position.z
        #rospy.loginfo(current_point)
        
def active_cb():
    rospy.loginfo("active callback")
def trunc(f, n):  
    # Truncates/pads a float f to n decimal places without rounding  
    slen = len('%.*f' % (n, f))  
    return float(str(f)[:slen])  


if __name__ == '__main__':  
    try:  
        
        global current_point
        current_point=Point(100, 100, 100)
        
        global orientation
        orientation=0.0
       
        MultiNav()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        rospy.loginfo("AMCL navigation test finished.")  
