
import rospy
from std_msgs.msg import String

def signal_callback(msg):
    if msg.data == "AI_GO_WHITE":
        pub.publish("22-18")
    elif msg.data == "AI_GO_BLACK":
        pub.publish("1x10;10x17")


if __name__ == "__main__":
    rospy.init_node("checkers_ai")

    pub = rospy.Publisher("/checkers/moves_msg", String, queue_size=50)

    sub = rospy.Subscriber("/checkers/ai_sig", String, signal_callback, queue_size=50)

    rospy.spin()
