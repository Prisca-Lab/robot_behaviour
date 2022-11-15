import rospy
import std_msgs.msg
from hri_msgs.msg import Expression
from abstract_behaviour import BehaviourMode


class Eyes(BehaviourMode):

    def __init__(self):
        name = self.__class__.__name__
        super().__init__(name)
        rospy.init_node(name, anonymous=True)
        self.eyes_pub = rospy.Publisher(
            '/eyes/expression', Expression, queue_size=10)

    # define publisher for changing robot facial expression
    def execute(self):
        expression_msg = Expression()
        expression_msg.header = std_msgs.msg.Header()
        expression_msg.expression = self.data
        rospy.loginfo(expression_msg)
        self.eyes_pub.publish(expression_msg)

    def stop(self):
        expression_msg = Expression()
        expression_msg.header = std_msgs.msg.Header()
        expression_msg.expression = "neutral"
        rospy.loginfo(expression_msg)
        self.eyes_pub.publish(expression_msg)


def main():
    eyes = Eyes()
    eyes.data = "happy"
    eyes.execute()


if __name__ == "__main__":
    main()
