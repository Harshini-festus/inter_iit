import rospy
from geometry_msgs.msg import Point, Twist

kp = 1
kd = 0
ki = 0
t_prev = 0   #last time stamp
e_prev = 0  #previous error
I = 0   #last integrating error
sample_pts=[]
#gps callback (topic -----/fix)
def callback(msg):
    lon = msg.longitude
    lat = msg.latitude
    t = rospy.Time.now()
    goal_lat, goal_lon = goal()
    pid(lon, lat, goal_lon, goal_lat,t)

# from 0,0 reach 10,0 ---gazebo frame
def goal():
    goal_lon, goal_lat = 0, 0
    

    return goal_lat, goal_lon


def pid(current_lon, current_lat, goal_lon, goal_lat,t):
    
    error_lon = goal_lon - current_lon
    error_lat = goal_lat - current_lat
    corr_lon = kp*(error_lon) 
    corr_lat=kp*(error_lat)
    correction=[corr_lon, corr_lat]

    
    # P_x = kp*error_lon
    # I = I + ki*e*(t - t_prev)
    # D = Kd*(e - e_prev)/(t - t_prev)


    
    return (correction)


rospy.init_node('pid') # defining the ros node - publish_node
goal_pos=rospy.Publisher("/pid/pos", Float64, queue_size=1)
rate =rospy.Rate(100) # frequency at which publishing    0.01 sec

while not rospy.is_shutdown():
    pub = rospy.Subscriber('/fix', Twist, callback)
    

    rate.sleep()