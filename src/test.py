import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from math import pow,atan2,sqrt,asin
import math
from nav_msgs.msg import Path
import time
from tf.transformations import euler_from_quaternion
#---------------------------------------------------------------

x = 0.0
y = 0.0 
theta = 0.0
route = []

#---------------------------------------------------------------
def call_back(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

#---------------------------------------------------------------


def dist(a,b):
    return(sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2))

#---------------------------------------------------------------

if __name__ == '__main__':
    try:
        
        rospy.init_node('sahayak_teleop_node', anonymous=True)
        print('Commencing Operation')

        #---------------------------------------------------------------
        # Location Of robot
        start = rospy.Subscriber("/ground_truth/state",Odometry, call_back)

        res= 0.030054
        # converting real-time coordinates to occupancy grid indices 
        col1, row1= int((x +50.01)/res) , int((y +50.01)/res)
        origin = (row1, col1)

        #---------------------------------------------------------------
        # Path
        path =  rospy.wait_for_message("TrajectoryPlannerROS/global_plan", Path)
        print(len(path.poses))

        path1 = []
        for i in range(len(path.poses)):
            xp = path.poses[i].pose.position.x
            yp = path.poses[i].pose.position.y

            colp, rowp= int((xp +50.01)/res) , int((yp +50.01)/res)
            path1.append((rowp,colp))
        print("Path Loaded")


            
        #---------------------------------------------------------------
        #declare velocity publisher
        vel_topic_1='/joint1_vel_controller/command'
        velocity_publisher_1 = rospy.Publisher(vel_topic_1, Float64, queue_size=10)

        vel_topic_2='/joint2_vel_controller/command'
        velocity_publisher_2 = rospy.Publisher(vel_topic_2, Float64, queue_size=10)

        vel_topic_3='/joint3_vel_controller/command'
        velocity_publisher_3 = rospy.Publisher(vel_topic_3, Float64, queue_size=10)

        vel_topic_4='/joint4_vel_controller/command'
        velocity_publisher_4 = rospy.Publisher(vel_topic_4, Float64, queue_size=10)

        rate= rospy.Rate(5)  

        j1 = Float64()
        j2 = Float64()
        j3 = Float64()
        j4 = Float64()
        #---------------------------------------------------------------


        R= 0.08
        L= 0.41

        point_path = int(len(path1)/5)

        z = 1
        print('Loop Started')
        while not rospy.is_shutdown():
            
            time.sleep(2)

            path_pose = path1[point_path]


            bot_pose = origin
            dis = dist(bot_pose , path_pose)

            inc_x = path_pose[0] - bot_pose[0]
            inc_y = path_pose[1] - bot_pose[1]
            phi = atan2(inc_y,inc_x)


            v_des = 0.2
            w_des = 0.8*phi

            v_r = v_des + L*w_des
            v_l = v_des - L*w_des
            v_l = v_l/R
            v_r = v_r/R

            if dis < 0.1:
                point_path = point_path + int(len(path1)/5)
                if point_path > len(path1):
                    v_r = 0
                    v_l = 0
            j1.data = v_r
            j2.data = -v_l
            j3.data = v_r
            j4.data = -v_l

            velocity_publisher_1.publish(j1)
            velocity_publisher_2.publish(j2)
            velocity_publisher_3.publish(j3)
            velocity_publisher_4.publish(j4)
            rate.sleep()


            print('Iteration ', z)
            z = z+1
            

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")




