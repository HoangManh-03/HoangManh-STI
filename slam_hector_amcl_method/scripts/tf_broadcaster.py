import rospy
import tf
from geometry_msgs.msg import TransformStamped

def publish_static_tf():
    rospy.init_node('static_tf_publisher')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "odom")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_static_tf()
    except rospy.ROSInterruptException:
        pass
