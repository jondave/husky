#!/usr/bin/python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from tf2_msgs.msg import TFMessage
import tf

pose = Pose()
def vicon_cb(data):
    global pose
    pose = data

def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z

rospy.init_node('pose_to_odom')

vicon_sub = rospy.Subscriber('/robot_pose', Pose, vicon_cb, queue_size=100)
odom_pub = rospy.Publisher('/odometry/robot_pose', Odometry, queue_size=100)

rate = rospy.Rate(50.0)
counter = 0
x = 0.
y = 0.

dt = 1./50.

while not rospy.is_shutdown():

    (v_roll,v_pitch,v_yaw) = quaternion_to_euler_angle(pose.orientation.w, pose.orientation.x , pose.orientation.y, pose.orientation.z)
    v_phi = float((v_roll))
    v_theta = float((v_pitch))
    v_psi = float((v_yaw))
    
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    yaw = math.radians(v_psi)

    if counter > 0:
        vel_x_world = (x - x_prev) / dt
        vel_y_world = (y - y_prev) / dt

        x_prev = x
        y_prev = y


        twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
        twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world


        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.header.stamp = rospy.Time.now()

        odom.pose.position.x = pose.position.x
        odom.pose.position.y = pose.position.y
        odom.pose.position.z = pose.position.z

        odom.pose.orientation.x = pose.orientation.x
        odom.pose.orientation.y = pose.orientation.y
        odom.pose.orientation.z = pose.orientation.z
        odom.pose.orientation.w = pose.orientation.w

        odom.twist.twist.linear.x = twist_x
        odom.twist.twist.linear.y = twist_y
        odom.twist.twist.linear.z = (z - z_prev) / dt
        z_prev = z

        odom.twist.twist.angular.x = 0.
        odom.twist.twist.angular.y = 0.
        odom.twist.twist.angular.z = 0.



        odom_pub.publish(odom)

        #br = tf.TransformBroadcaster()
        #br.sendTransform((x,y,z),[pose.orientation.x, pose.orientation.y,pose.orientation.z,pose.orientation.w],rospy.Time.now(), "base_link","odom")

    else:
        x_prev = x
        y_prev = y
        z_prev = z
        counter += 1



    rate.sleep()
