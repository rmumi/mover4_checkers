
import rospy
from std_msgs.msg import String

def signal_callback(msg):
    if msg.data == "AI_GO_WHITE":
        pub.publish("1-10;10-17;14-18")
    elif msg.data == "AI_GO_BLACK":
        pub.publish("18-14;17-10;10-1")


if __name__ == "__main__":
    rospy.init_node("checkers_ai")

    pub = rospy.Publisher("/checkers/moves_msg", String, queue_size=50)

    sub = rospy.Subscriber("/checkers/ai_sig", String, signal_callback, queue_size=50)

    rospy.spin()
