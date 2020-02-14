import os
import signal
from subprocess import Popen
from threading import Thread

import rospy
import roslaunch

import rosbag
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from rosgraph_msgs.msg import Clock

from marvelmind_nav.msg import hedge_pos_a

from bdi.robot_control import RobotControl


def run(cmd, stdout, stderr):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    stdout : str or subprocess.PIPE object
        Destination of stdout output.
    stderr : str or subprocess.PIPE object
        Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr, shell=False,
                 preexec_fn=os.setsid)


def start_process(cmd, typ, start_time, dpath_logs):
    """Start a subprocess with the given command `cmd`.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    typ : str
        Type of subprocess. This will be included in the logs' file names.
    start_time : str
        Datetime string, will be included in the logs' file names as well as
        the resulting bag's name.
    dpath_logs :
        Path to log direcotry.

    Returns
    -------
    A subprocess.Popen instance.
    """
    print('Starting', typ.upper())
    stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
    with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
        return run(cmd, stdout=out, stderr=err)
￼
# x TODO: storage of rosbag action data (start, end, filename, classification)
# x TODO: keeping track of time (subscribing to the simulation clock, launching events at predefined time steps)
# x TODO: start rosbag from experimenter
# x TODO: in worldstate: add trajectory calculus, generate labels for human movement relative to his node location as seen from the robot's perspective
# x TODO: generate human behavior events from observed movement
# x TODO: add distance metric and emergency stop

# . TODO: check reasoning
# . TODO: check pose detection
# . TODO: check experimenter

# . TODO: add pose for picking berries
# . TODO: add pose for call
# . TODO: add pose for cancel
# . TODO: add pose for stop
# . TODO: in bdi system: pause, start simulation, change simulation realtime factor

class Config(Thread):

    def __init__(self):
        self.parameters = {}
        self.launch_files = ["/home/rasbery/catkin_ws/src/rasberry_hri/launch/extraction.launch", "/home/rasbery/catkin_ws/src/rasberry_hri/launch/hri_agent.launch"]
        self.actions = {120: {"start": None,
                            "duration": None,
                            "filename": None,
                            "label": None},
                        240: {"start": None,
                            "duration": None,
                            "filename": None,
                            "label": None},
                            }
        self.action_list = []
        self.picker_pose = hedge_pos_a()
        self.picker_pose.address = 2 #PICKER_ID
        # TODO set robot and picker starting locations
        self.picker_pose.x_m = 1.0
        self.picker_pose.y_m = 1.0

        self.robot_pose = PoseWithCovarianceStamped()
        self.robot_pose.position.x = 1.0
        self.robot_pose.position.y = 1.0
        self.robot_pose.position.z = 0.0


    def add_parameter(self, label, values):
        if not isinstance(values, list):
            values = [values]
        self.parameters[label] = values

    def get_next_action_time(self):
        return self.next_action_time[0]


    def get_next_action(self):
        return self.actions[self.action_list.pop()]


    def get_parameter_set(self):
        try:
            return self.parameter_set
        except AttributeError:
            keys, values = zip(*self.parameters.items())
            self.parameter_set = [dict(zip(keys,v)) for v in itertools.product(*values)]
            return self.parameter_set


    def setup(self, robot_pose_publisher, picker_pose_publisher):
        # add config specific stuff to reasoning system, <- do this as defined by rosparameters inside the respective nodes instead
        robot_pose_publisher.publish(self.robot_pose)
        picker_pose_publisher.publish(self.picker_pose)


    def run(self):
        # scheduler = start_process(['/bin/bash', "/opt/ros/kinetic/bin/rosbag rasberry_hri scheduler"],
                               # 'node', start_time_string,
        #                        dpath_logs)
        self.launches = []
        for lauch_file in self.launch_files:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch  = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
            self.launches.append(launch)
            launch.start()


    def shutdown(self):
        for launch in self.launches:
            launch.shutdown()




class Experimenter:

    def __init__(self, robot_id):
        self.configs = []
        self.running_bags = []
        self.current_config = None
        self.robot_id = robot_id
        self.robco = RobotControl(self.robot_id)
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        rospy.loginfo("EXP: Waiting for gazebo services")
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        rospy.loginfo("EXP: Found gazebo services")
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
        self.robot_pose_publisher = rospy.Publisher('/{:}/initialpose'.format(robot_id), PoseWithCovarianceStamped, queue_size=10)
        self.picker_pose_publisher = rospy.Publisher('/hedge_pos_a', hedge_pos_a, queue_size=10)
        # p_ros_core = start_process(['/opt/ros/kinetic/bin/roscore'],
        #                         'ros', start_time, dpath_logs)
        # print('PGID ROS: ', os.getpgid(p_ros_core.pid))


    def reset(self):
        self.robco.cancel_movement()
        # ?? stop all running rosbags?
        self.current_config.shutdown()
        self.terminate_bags()
        self.reset_simulation()


    def run(self):
        for config in self.configs:
            self.current_config = config
            for parameters in config.get_parameter_set():
                # this is one experiment run
                self.reset()
                for label, value in parameters:
                    rospy.set_param(label, value)
                config.setup(self.robot_pose_publisher, self.picker_pose_publisher)
                config.start()
                config.join()


    def terminate_bags(self):
        for rosbag in self.running_bags:
            rosbag.terminate()
            rospy.sleep(1)
            if rosbag.poll() is None:
                rosbag.kill()
        self.running_bags = []


    def clock_callback(self, msg):
        clock = msg.clock.to_sec()
        difference = clock - last_clock
        index = 0
        while index < len(self.running_bags):
            if not self.running_bags[index].poll() is None:
                del self.running_bags[i]
            else:
                index+=1
        while clock+difference >= self.current_config.get_next_action_time():
            action = self.current_config.get_next_action()
            rosbag = start_process(['/bin/bash', "/opt/ros/kinetic/bin/rosbag play --start={:f} --duration={:f} {:}".format(action["start"], action["duration"], action["filename"])],
                                   'rosbag {:} {:}'.format(action["label"], action["filename"]), action["start"],
                                   dpath_logs)
            self.running_bags.append(rosbag)
