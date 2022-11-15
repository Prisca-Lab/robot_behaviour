# coding=utf-8
from eyes_reproducer import Eyes
from gesture_reproducer import Gesture
from speech_reproducer import Speech

from abstract_behaviour import BehaviourMode
import rospy

class PlayBehaviour:
    """
  This is a module to reproduce a robot action combining speech,
  eyes expression and gesture.
  """

    def __init__(self, modes = [BehaviourMode]):
        for m in modes:
            assert isinstance(m, BehaviourMode), "mode is not a BehaviourMode"

        self.modes = modes
        rospy.init_node("manager", anonymous=True)

    def run(self):
        """
    it reproduces a complex robot behaviour by
    combining multi-modal interactions such as voice,
    gesture and eyes expression

    sentence: the text the robot has to reproduce
    gesture_name: the name of the prerecorded motion
    the robot has to play using play_motion
    eyes_expression_name: the name of the eyes expression
    """

        for m in self.modes:
            m.execute()

    def stop(self):
        for m in self.modes:
            m.stop()



def main():
    speech = Speech("it_IT")
    gesture = Gesture()
    eyes = Eyes()

    modes = [speech, gesture, eyes]
    behaviour = PlayBehaviour(modes)

    speech.data = "Ciao sono Antonio"
    gesture.data = "nodding.yaml"
    eyes.data = "sad"

    behaviour.run()


if __name__ == "__main__":
    main()
