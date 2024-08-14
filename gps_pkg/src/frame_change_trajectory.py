#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion

class TF_publisher:
    def __init__(self):
        #initialisation du noeud ROS et du broadcaster de transformation
        rospy.init_node('tf_publisher',  anonymous=True)
        self.br = tf2_ros.TransformBroadcaster()
        self.point_sub = rospy.Subscriber('/gps_rover/fix_try', PoseStamped, self.point_callback)
        self.rate = rospy.Rate(10)
        self.vector = Vector3()
        self.vector.x = 0
        self.vector.y = 0
        self.vector.z = 0
        self.orientation = Quaternion()
        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.orientation.w = 1
        self.time = rospy.Time.now()

    def point_callback(self, msg):
        self.vector.x = msg.pose.position.x
        self.vector.y = msg.pose.position.y
        self.orientation = msg.pose.orientation
        self.time = msg.header.stamp

    def broadcast(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # Creation et initialisation du message TransformStam
            t = TransformStamped()
            t.header.stamp = self.time
            t.header.frame_id = "map"
            t.child_frame_id = "trajectory"
            t.transform.translation.x = self.vector.x
            t.transform.translation.y = self.vector.y
            t.transform.translation.z = self.vector.z
            t.transform.rotation = self.orientation
            
            # Envoyer la transformation
            self.br.sendTransform(t)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        pub = TF_publisher()
        pub.broadcast()
    except rospy.ROSInterruptException:
        pass
