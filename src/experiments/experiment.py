#!/usr/bin/env python
import os

import base64

import paramiko
import rospy
import roslaunch
import rosparam
import rosnode
# import dynamic_reconfigure.client

from std_srvs.srv import Empty  # , Trigger

# from robot_localization. srv import ToggleFilterProcessing
from std_msgs.msg import String

# from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState  # , GetModelState
from rasberry_hri.msg import Action

# from config import Config
from bdi.robot_control import RobotControl
from common.parameters import NS, BEHAVIOURS, CONFIG_DIRECTORY, STATE_DIRECTORY

import warnings
warnings.filterwarnings(action='ignore', module='.*paramiko.*')

key = paramiko.RSAKey(
    data=base64.b64decode(
        b"AAAAB3NzaC1yc2EAAAADAQABAAABAQC9/fxTMxUkCY0+27UqYXJM+RZfJjDBeCgqYb28P5iq0Kqry98Ke0Af703yyJPfaVlRG8PUkn95k+1dh1xQL8nzWeIs9oxTOwjpOliqAn2ajQ/v2J73E2wQyHhGfRWLQ+GVzO6ZU8u48mybWxOKPsk6ETG/0eQdes+zNvI7Q0K2ZryTv/BzfWzGite2I3YTDHMGjg94BdWsQJ3t+leV29nsn6eH4vYliFJcDJRexck/FvA2j3n3eqh/ahP2UTFChna3BS3EGfGTaFUaqxJyxijXtXAkZd6OGweD1D6VREX50/JwjtCbIeYQTz+pz7dw67dBQHU9yKX40x+iR813KCgt"
    )
)

# command = """tmux send-keys -t "robot_local.0" C-c C-m 'roslaunch rasberry_navigation rasberry_localisation_multisim.launch use_imu:="$USE_IMU" publish_tf:="$EKF_PUBLISH_TF" robot_name:=$ROBOT_NAME_1 initial_pose_x:=$ROBOT_POS_X_1 initial_pose_y:=$ROBOT_POS_Y_1 initial_pose_a:=$ROBOT_POS_A_1' C-m"""

# 102: 8.675 4.65
# 103: 11.649 4.64
# 104: 14.12 4.612
# 105: 17.061 4.609
# 106: 19.997 4.568


def get_command(x, y):
    return """tmux send-keys -t "robot_local.0" C-c C-m 'roslaunch rasberry_navigation rasberry_localisation_multisim.launch use_imu:="$USE_IMU" publish_tf:="$EKF_PUBLISH_TF" robot_name:=$ROBOT_NAME_1 initial_pose_x:={:0.3f} initial_pose_y:={:0.3f} initial_pose_a:=$ROBOT_POS_A_1' C-m""".format(
        x, y
    )


LOG_PATH = "."


class Experiment:
    def __init__(self, config):
        self.config = config

        # rospy.set_param("{}/experiment_id".format(NS), self.config["experiment_id"])
        # super(Config, self).__init__()
        rospy.loginfo(
            "EXP: Initializing Experiment: {}".format(config["experiment_label"])
        )
        self.launch_files = [
            "/home/rasberry/catkin_ws/src/rasberry_hri/launch/picker_mover.launch",
            "/home/rasberry/catkin_ws/src/rasberry_hri/launch/hri_agent.launch"
        ]
        self.set_model_state = self.create_service_proxy(
            "/gazebo/set_model_state", SetModelState, local=False
        )
        rospy.wait_for_service("/gazebo/set_model_state")
        state_msg = ModelState()
        state_msg.model_name = self.config["robot"]
        # state_msg.pose.position.x = 11.41  # 14.12 #11.649
        # state_msg.pose.position.y = 4.6285  # 4.63 #4.64
        x = self.config["robot_position"][0]
        y = self.config["robot_position"][1]
        rospy.loginfo("Setting robot to {:.2f}, {:.2f}".format(x, y))
        state_msg.pose.position.x = x  # 14.12 #11.649
        state_msg.pose.position.y = y  # 4.63 #4.64
        state_msg.pose.orientation.w = 1
        state_msg.reference_frame = "map"

        self.set_model_state(state_msg)
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # self.ssh_client.get_host_keys().add('10.10.10.1', 'ssh-rsa', key)
        self.ssh_client.connect("simulator", username="rasberry")
        stdin, stdout, stderr = self.ssh_client.exec_command(get_command(x, y))
        self.ssh_client.close()
        rospy.sleep(3)
        self.is_finished = False

        self.robco = RobotControl(self.config["robot"])

        self.initial_pose_publisher = rospy.Publisher(
            "/{:}/initialpose".format(self.config["robot"]),
            PoseWithCovarianceStamped,
            queue_size=10,
        )
        self.picker_pose_publisher = rospy.Publisher(
            "/picker_mover", String, queue_size=10
        )
        self.human_action_publisher = rospy.Publisher(
            "/human_actions", Action, queue_size=10
        )
        self.robot_pose_publisher = rospy.Publisher(
            "/{:}/set_pose".format(self.config["robot"]),
            PoseWithCovarianceStamped,
            queue_size=10,
        )
        # self.camera_publisher = rospy.Publisher(
        #       "/camera/color/image_raw", Image, queue_size=10)

        # self.bag_paths = convert_bags(self.config)
        # self.bags = self.config.get_bag_paths()
        # self.running_bags = []



        rospy.loginfo("EXP: Initialization finished")
        # reconfigure robot speed
        # client = dynamic_reconfigure.client.Client("/{:}/move_base/DWAPlannerROS".format(self.config["robot"]), timeout=4)
        # client.update_configuration(
        # {"max_vel_x":1.5})
        # {
        # "int_param":x,
        # "double_param":(1/(x+1)),
        # "str_param":str(rospy.get_rostime()),
        # "bool_param":b, "size":1})

    def create_service_proxy(self, topic, type=Empty, local=True):
        if local:
            topic = "/{:}/{:}".format(self.config["robot"], topic)
        rospy.loginfo("EXP: Waiting for service {}".format(topic))
        service = rospy.ServiceProxy("{:}".format(topic), type)
        rospy.wait_for_service(topic)
        rospy.loginfo("EXP: Found service {}".format(topic))
        return service

    def initial_pose_callback(self, msg):
        rospy.loginfo("EXP: Got initial pose")
        rospy.loginfo(msg)

    def launch_services(self):
        # scheduler = start_process(['/bin/bash', "/opt/ros/kinetic/bin/rosbag rasberry_hri scheduler"],
        # 'node', start_time_string,
        #                        dpath_logs)
        run_id = roslaunch.rlutil.get_or_generate_uuid(None, True)
        # roslaunch.configure_logging(run_id)
        # config = roslaunch.ROSLaunchConfig()
        # config.set_master(roslaunch.core.Master())
        # # node = roslaunch.core.Node(package, executable)
        # # runner = roslaunch.ROSLaunchRunner(self.run_id, config, server_uri="http://localhost:11311")
        self.launch = roslaunch.parent.ROSLaunchParent(
            run_id, self.launch_files, is_core=False
        )
        self.launch.start()

    def spin(self):
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException(
                "client code must call rospy.init_node() first"
            )
        rospy.logdebug(
            "node[%s, %s] entering spin(), pid[%s]",
            rospy.core.get_caller_id(),
            rospy.core.get_node_uri(),
            os.getpid(),
        )
        try:
            while not rospy.core.is_shutdown() and not self.is_finished:
                self.launch.spin_once()
                rospy.rostime.wallsleep(0.01)
            rospy.logwarn("EXP: at end of spin")
        except KeyboardInterrupt:
            rospy.logwarn("EXP: keyboard interrupt")
        except rospy.ROSInterruptException:
            rospy.logwarn("EXP: remote interrupt")
        finally:
            self.shutdown()

    def set_parameters(self):
        for key, value in self.config.items():
            key = "{}/{}".format(NS, key)
            rospy.loginfo("Set parameter: {}:{}".format(key, value))
            rospy.set_param(key, value)
        param_file = os.path.join(CONFIG_DIRECTORY, STATE_DIRECTORY, self.config["run_id"] + ".param")
        rosparam.dump_params(param_file, NS, verbose=False)

    def unset_parameters(self):
        for key in self.config.keys():
            try:
                key = "{}/{}".format(NS, key)
                rospy.delete_param(key)
            except KeyError:
                pass

    def setup(self):
        self.set_parameters()
        key = "{}/{}".format(NS, "behaviours")
        rospy.set_param(key, BEHAVIOURS[self.config["behaviour"]])
        self.launch_services()
        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)
        rospy.loginfo("EXP: Experiment setup finished")

        # TODO: wait for everything to have started
        self.start_clock = int(rospy.get_time() + 600)
        running_nodes = rosnode.get_node_names()
        while ("/thorvald_001/picker_mover" not in running_nodes
            or '/thorvald_001/action_recognition_node' not in running_nodes
            or '/thorvald_001/scheduler' not in running_nodes
            or '/thorvald_001/openpose' not in running_nodes):
            rospy.sleep(1)
            running_nodes = rosnode.get_node_names()
        self.start_clock = int(rospy.get_time() + 3)
        rospy.set_param("{}/start_time".format(NS), self.start_clock)
        rospy.loginfo(
            "EXP: Experiment starting at: {}".format(self.start_clock)
        )
        # self.reset_simulation()
        # self.robot_pose_publisher.publish(self.robot_pose)
        # add config specific stuff to reasoning system, <- do this as defined by rosparameters inside the respective nodes instead
        # picker_pose_publisher.publish(self.picker_pose)

    def clock_callback(self, msg):
        if msg.clock.nsecs == 0:
            clock = msg.clock.to_sec()
            # rospy.loginfo("Now {:}, Start {:}, End {:}"
            #               .format(clock, self.start_clock,
            #                       self.config["termination_time"]))
            if clock >= self.config["termination_time"] + self.start_clock:
                if not self.is_finished:
                    rospy.loginfo("EXP: Timeout condition reached")
                    self.is_finished = True
                return

            # index = 0
            # while index < len(self.running_bags):
            #     if not self.running_bags[index] is None:
            #         del self.running_bags[i]
            #     else:
            #         index+=1

            # ----
            # difference = clock - self.last_clock
            # self.last_clock = clock
            # try:
            #     while (
            #         clock
            #         == self.config.get_next_behaviour_time() + self.start_clock
            #     ):
            #         behaviour = self.config.get_next_behaviour()
            #         if behaviour["type"] == "rosbag":
            #             rospy.loginfo(
            #                 "EXP: Playing rosbag '{}'".format(
            #                     behaviour["filename"]
            #                 )
            #             )
            #             player = Player(
            #                 behaviour["filename"],
            #                 topics=behaviour["topics"],
            #                 start_time=behaviour["start"],
            #                 duration=behaviour["duration"],
            #             )
            #             player.start()
            #             # for topic, outmsg, t in bag.read_messages(topics=behaviour["topics"]):
            #             #     secs = t.to_sec()
            #             #     if secs >= start_time and secs <= end_time:
            #             #         outmsg.header.stamp.secs += clock
            #             #         # self.camera_publisher.publish(outmsg)
            #             #         self.human_action_publisher.publish(outmsg)
            #             self.running_bags.append(player)
            #         elif behaviour["type"] == "message":
            #             if behaviour["target"] == "picker_movement":
            #                 rospy.loginfo(
            #                     "EXP: Sending Movement message '{}'".format(
            #                         behaviour["message"]
            #                     )
            #                 )
            #                 self.picker_pose_publisher.publish(
            #                     behaviour["message"]
            #                 )
            #             elif behaviour["target"] == "action_label":
            #                 rospy.loginfo(
            #                     "EXP: Sending Behaviour message '{}'".format(
            #                         behaviour["message"]
            #                     )
            #                 )
            #                 outmsg = Action()
            #                 outmsg.action = behaviour["message"]
            #                 outmsg.id = rospy.get_param(
            #                     "{}/hri/target_picker".format(
            #                         self.config.robot_id
            #                     ),
            #                     "Picker02",
            #                 )
            #                 self.human_action_publisher.publish(outmsg)
            # except AttributeError as err:
            #     rospy.logerr("EXP: clock_callback - {}".format(err))

    def shutdown(self):
        rospy.loginfo("EXP: Shutting dow")
        self.robco.cancel_movement()
        self.launch.shutdown()
        self.clock_sub.unregister()
        # self.terminate_bags()
        # for bag in self.bags.values():
        #     bag.close()
        # wait for launch to have shutdown
        self.unset_parameters()
        rospy.loginfo("EXP: Experiment ended")
