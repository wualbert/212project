import rospy
from std_msgs.msg import Float32

class Add:
    def __init__(self):
        rospy.Subscriber('/a', Float32, self.callback)
        self.pub = rospy.Publisher('/b', Float32, queue_size = 10)
    def callback(self, msg):
        result = Float32()
        print msg.data
        result.data= msg.data + 1.
        self.pub.publish(result)

if __name__ == '__main__':
    rospy.init_node('node')
    node = Add()
    rospy.spin()
