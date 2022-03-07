import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
#goal_x, goal_y let us make it as rosparam

x_current=0
y_current=0
theta=0
v_x=0
v_y=0
w_z=0
trial=0
controlled_vel=Twist()
hz=100
t_stamp=1/hz
alpha=0
goal=[0,0]

#now we start from 0,0 and want to reach 5,0
rospy.set_param('final_goal', [5,0])
rospy.set_param('sampled_pts', [(1,0),(2,0),(3,0),(4,0)])
rospy.set_param('alpha', 0)
rospy.set_param('goal_iteration', 0) #i===iterations of sampled pts done so far
final=rospy.get_param('final_goal')
rate=rospy.Rate(hz) #time_stamp=0.01

#all the real world values will be stored by this callback fuunction.
def odom_callback(msg):
    x_current=msg.pose.pose.position.x
    y_current=msg.pose.pose.position.y
    thet_current=msg.pose.pose.orientation.z
    v_x=msg.twist.twist.linear.x
    v_y=msg.twist.twist.linear.y
    w_z=msg.twist.twist.angular.z

w_last_error=0
vx_last_error=0

def controller():
    # v_r=math.sqrt(math.pow(v_x, 2)+math.pow(v_y, 2))
    theta_head=math.atan(v_y/v_x) #in radians w.r.t positive x axis
    
    w_interest=-(theta-alpha)/t_stamp
    vx_interest=(goal[0]-x_current)/t_stamp
    
    w_error=w_z-w_interest
    vx_error=v_x-vx_interest

    kp_w,kp_vx=0,0
    ki_w,ki_vx=0,0
    kd_w,kd_vx=0,0

    #ackerman controller 
    #controller for angular in z
    p_w=w_error
    i_w+=w_error
    d_w=w_error-w_last_error
    w_last_error=w_error
    w_corrected = w_interest + p_w*kp_w + i_w*ki_w + d_w*kd_w
    controlled_vel.angular.z = w_corrected

    #controller for linear x
    p_vx=vx_error
    i_vx+=vx_error
    d_vx=vx_error-vx_last_error
    vx_last_error=vx_error
    vx_corrected = vx_interest + p_vx*kp_vx + i_vx*ki_vx + d_vx*kd_vx
    controlled_vel.linear.x = vx_corrected
    
       
while not rospy.is_shutdown():
    rospy.init_node("ackerman_controller")
    odom_subscriber=rospy.Subscriber('/odom', odom_callback)
    velocity_pub=rospy.Publisher('/cmd_vel', Twist())
    
    (goal_x,goal_y)=rospy.get_param(sampled_pts[trial])
    goal=[goal_x,goal_y]

    velocity_pub.publish(controlled_vel)

    if x_current==goal[0] and y_current==goal[1] and trial<4:
        trial=rospy.get_param('goal_iteration')
        sample_pts=rospy.get_param('sampled_pts')
        (x_prev_goal,y_prev_goal)=sample_pts[trial]
        trial+=1
        rospy.set_param('goal_iteration', trial)
        (x_next_goal,y_next_goal)=sample_pts[trial]
        alpha_line=math.atan((y_next_goal-y_prev_goal)/(x_next_goal-x_prev_goal))
        rospy.set_param('alpha', alpha_line)
    
    
    rospy.signal_shutdown(x_current==final[0] and y_current==final[1])