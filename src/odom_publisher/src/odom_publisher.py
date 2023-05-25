#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Twist
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import tf

rospy.init_node('odom_pub')

br = tf.TransformBroadcaster()
tf_sub = tf.TransformListener()


# odom_pub=rospy.Publisher ('/odom', Odometry, queue_size=1)

rospy.wait_for_service ('/gazebo/get_model_state')
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

odom=Odometry()
header = Header()
twist = Twist()
header.frame_id='/odom'
odom.child_frame_id = "/dummy"


model = GetModelStateRequest()
model.model_name='rr_robot'

r = rospy.Rate(50)

while not rospy.is_shutdown():
    result = get_model_srv(model)

    header.stamp = rospy.Time.now()
    odom.pose.pose = result.pose

    br.sendTransform((result.pose.position.x, result.pose.position.y, result.pose.position.z),
                (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z, result.pose.orientation.w),
                rospy.Time.now(),
                "dummy",
                "odom")    

    linear_stamped = Vector3Stamped(header=header, vector=result.twist.linear)
    angular_stamped = Vector3Stamped(header=header, vector=result.twist.angular)


    try:
        twist.linear = tf_sub.transformVector3("dummy", linear_stamped).vector
        twist.angular = tf_sub.transformVector3("dummy", angular_stamped).vector
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        r.sleep()
        continue

    odom.twist.twist = twist

    odom.header = header

    # odom_pub.publish(odom)

    r.sleep()


