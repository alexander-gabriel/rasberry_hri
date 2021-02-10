
import rospy

from geometry_msgs.msg import Twist

from commandInterface import CommandInterface


class CommandVelocity(CommandInterface):

    def __init__(self):
        self.frequency = rospy.get_param("~frequency", 30.0)
        self.commanded_speed = rospy.get_param("~commanded_speed", 0.5)
        self.controller_frequency = rospy.get_param("~controller_frequency", 10.0)
        self.vel_pub = rospy.Publisher("nav_vel", Twist, queue_size=1)


    def command_velocity(self, msg):
        start_time = rospy.Time.now()
        rospy.logwarn("Publishing to vel ")
        while (rospy.Time.now() - start_time).to_sec() < 1 / self.frequency:
            self.vel_pub.publish(msg)
            rospy.sleep(1 / self.controller_frequency)


    def move_forward(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = self.commanded_speed
        self.command_velocity(cmd_vel)


    def move_backward(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = - self.commanded_speed
        self.command_velocity(cmd_vel)


    def move_right(self):
        cmd_vel = Twist()
        cmd_vel.linear.y = - self.commanded_speed
        self.command_velocity(cmd_vel)


    def move_left(self):
        cmd_vel = Twist()
        cmd_vel.linear.y = self.commanded_speed
        self.command_velocity(cmd_vel)


    def turn_right(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = - self.commanded_speed
        self.command_velocity(cmd_vel)


    def turn_left(self):
        cmd_vel = Twist()
        cmd_vel.angular.z = self.commanded_speed
        self.command_velocity(cmd_vel)
