#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def listener():
    # Khoi tao node ROS
    rospy.init_node('tf_listener_node', anonymous=True)
    
    # Tao mot TransformListener
    listener = tf.TransformListener()

    # Dinh nghia frame
    map_frame = "/frame_map_nav350"
    base_frame = "/nav"

    rate = rospy.Rate(10)  # Tan suat 10 Hz

    while not rospy.is_shutdown():
        try:
            # Lay thoi gian hien tai
            now = rospy.Time.now()
            
            # Doi cho phép bien doi có san
            listener.waitForTransform(map_frame, base_frame, now, rospy.Duration(4.0))

            # Nhan phép bien doi tu map_frame den base_frame
            (trans, rot) = listener.lookupTransform(map_frame, base_frame, now)
            
            # In ra ket qua
            rospy.loginfo("Translation: %s", str(trans))
            rospy.loginfo("Rotation: %s", str(rot))

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: %s", str(e))
        
        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

