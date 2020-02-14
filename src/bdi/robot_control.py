import rospy
import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

class RobotControl:


    def __init__(self, robot_id="thorvald_014"):
        rospy.loginfo("RCO: Initializing Robot Control")
        self.robot_id = robot_id
        rospy.loginfo("RCO: Waiting for Robot Control Server...")
        self.robot_control = actionlib.SimpleActionClient('/{:}/topological_navigation'.format(self.robot_id), GotoNodeAction)
        self.robot_control.wait_for_server()
        rospy.loginfo("RCO: Found Robot Control Server")


    def move_to(self, goal):
        rospy.loginfo("RCO: Requesting Navigation to: {:}".format(goal))
        # rospy.get_name()
        navgoal = GotoNodeGoal()
        navgoal.target = goal
        #navgoal.origin = orig

        # Sends the goal to the action server.
        self.robot_control.send_goal(navgoal)
        # self.robot_control.send_goal_and_wait(navgoal)


    # Waits for the server to finish performing the action.
    def get_result(self):
        self.robot_control.wait_for_result()
        return self.robot_control.get_result()  # A FibonacciResult

    def cancel_movement(self):
        self.robot_control.cancel_all_goals()
