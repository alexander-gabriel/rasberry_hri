import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class RobotControl:


    def __init__(self, robot_control, robot_id="thorvald_014"):
        rospy.loginfo("RC: Initializing Robot Control")
        self.robot_id = robot_id
        self.robot_control = robot_control


    def move_to(self, goal):
        rospy.loginfo("RC: Requesting Navigation to {:}".format(goal))
        navgoal = GotoNodeGoal()
        navgoal.target = goal
        #navgoal.origin = orig

        # Sends the goal to the action server.
        self.robot_control.send_goal_and_wait(navgoal)
        #,self.done_cb, self.active_cb, self.feedback_cb)

        # Waits for the server to finish performing the action.
        # self.robot_control.wait_for_result()

        # Prints out the result of executing the action
        return self.robot_control.get_result()  # A FibonacciResult


    def cancel_movement(self):
        self.robot_control.cancel_all_goals()
