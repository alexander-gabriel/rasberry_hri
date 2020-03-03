#!/usr/bin/env python
import os
import sys
from threading import Thread

import rospy
import rosbag
import roslaunch
import dynamic_reconfigure.client

from std_srvs.srv import Empty
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from rosgraph_msgs.msg import Clock
from rasberry_hri.msg import Action

from utils import start_process
from config import Config
from bdi.robot_control import RobotControl

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, GetModelState



SLEEP = 0.01
LOG_PATH = "."

def get_rosbag_from_file(filename, mode='r'):
    try:
        bag = rosbag.Bag(filename, mode)
        return bag
    except Exception as e:
        print("Failed to get rosbag topics info from file {:} with exception: '{:}'".format(filename, e))
        return None



class Player(Thread):

    def __init__(self, filename, topics=[], start_time=None, duration=None):
        super(Player, self).__init__()
        self.bag = get_rosbag_from_file(filename)
        bag_start_time = self.bag.get_start_time()
        self.msgs = self.bag.read_messages(topics=topics)

        self.pubs = {}
        self.duration = duration
        start_time = None
        self.start_timestamp = bag_start_time + start_time if start_time is not None else bag_start_time


    def run(self):
        for topic, msg, timestamp in self.msgs:
            # topic = "/human_actions_old"
            secs = timestamp.to_sec()
            #     if secs >= start_time and secs <= end_time:
            if topic not in self.pubs:
                self.pubs[topic] = rospy.Publisher(topic, msg.__class__, queue_size=10)
                rospy.sleep(0.05)
            if (secs >= self.start_timestamp) and (self.duration is None or secs < self.start_timestamp + self.duration):
                self.pubs[topic].publish(msg)
                rospy.sleep(SLEEP)


    def close(self):
        self.bag.close()


    def terminate(self):
        self.bag.terminate()
        rospy.sleep(1)
        if self.bag.poll() is None:
            self.bag.kill()
        self.bag.close()



def convert_bags(config):
    out_paths = []
    for key, behaviour in config.behaviours.items():
        if behaviour["type"] == "rosbag":
            for topic in behaviour["topics"]:
                msgs = []
                start_time = behaviour["start"]
                end_time = start_time + behaviour["duration"]
                with rosbag.Bag(behaviour["filename"], "r") as in_bag:
                    bag_time = in_bag.get_start_time()
                    for topic, msg, t in in_bag.read_messages(topics=[topic]):
                        secs = t.to_sec() - bag_time - start_time
                        if secs >= 0 and secs <= end_time - start_time:
                            msg.header.stamp = rospy.Time.from_sec(secs)
                            msgs.append(msg)
                msgs = sorted(msgs, key=lambda msg: msg.header.seq)
                try:
                    os.mkdir(behaviour["filename"][:-4])
                except:
                    pass
                out_path = os.path.join(behaviour["filename"][:-4], "{:}-{:}.bag".format(behaviour["label"],start_time))
                behaviour["filename"] = out_path
                behaviour["start"] = 0
                out_paths.append(out_path)
                with rosbag.Bag(out_path, 'w') as out_bag:
                    for msg in msgs:
                            out_bag.write(topic, msg, msg.header.stamp)
            return out_paths



class Experiment():

    def __init__(self, parameters, config):
        # super(Config, self).__init__()
        rospy.loginfo("EXP: Starting Experiment")
        self.is_finished = False
        self.parameters = parameters
        self.config = config
        self.robco = RobotControl(self.config.robot_id)
        self.last_clock = rospy.get_time()
        self.start_clock = int(self.last_clock+3)
        self.request_nomotion_update = rospy.ServiceProxy('/{:}/request_nomotion_update'.format(self.config.robot_id), Empty)
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo("EXP: Waiting for gazebo services")
        rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        rospy.loginfo("EXP: Found gazebo services")
        self.initial_pose_publisher = rospy.Publisher('/{:}/initialpose'.format(self.config.robot_id), PoseWithCovarianceStamped, queue_size=10)
        self.picker_pose_publisher = rospy.Publisher('/picker_mover', String, queue_size=10)
        self.human_action_publisher = rospy.Publisher('/human_actions', Action, queue_size=10)
        self.robot_pose_publisher = rospy.Publisher('/{:}/set_pose'.format(self.config.robot_id), PoseWithCovarianceStamped, queue_size=10)
        self.camera_publisher = rospy.Publisher("/camera/color/image_raw", Image, queue_size=10)

        # self.bag_paths = convert_bags(self.config)
        self.bags = self.config.get_bag_paths()
        self.running_bags = []

        # reconfigure robot speed
        client = dynamic_reconfigure.client.Client("/{:}/move_base/DWAPlannerROS".format(self.config.robot_id), timeout=4)
        client.update_configuration(
        {"max_vel_x":1.5})
                        # {
                        # "int_param":x,
                        # "double_param":(1/(x+1)),
                        # "str_param":str(rospy.get_rostime()),
                        # "bool_param":b, "size":1})

        # self.picker_pose.x_m = 19.997
        # self.picker_pose.y_m = 4.568

        #102: 8.675 4.65
        #103: 11.649 4.64
        #104: 14.12 4.612
        #105: 17.061 4.609
        #106: 19.997 4.568

        # rospy.wait_for_service('/gazebo/set_model_state')
        # rospy.wait_for_service('/{:}/request_nomotion_update'.format(self.config.robot_id))
        # rospy.wait_for_service('/gazebo/get_model_state')
        # resp = self.get_model_state(self.config.robot_id, "")
        # state_msg = ModelState()
        # state_msg.model_name = self.config.robot_id
        # state_msg.pose = resp.pose
        # state_msg.pose.position.x = 11.649 #11.649
        # state_msg.pose.position.y = 4.62 #4.64
        # robot_pose = PoseWithCovarianceStamped()
        # robot_pose.pose.pose.position.x = 11.649
        # robot_pose.pose.pose.position.y = 4.64
        # resp = self.set_model_state( state_msg )
        # self.initial_pose_publisher.publish(robot_pose)
        # self.request_nomotion_update()

        # rospy.logwarn(resp)



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
        self.launch = roslaunch.parent.ROSLaunchParent(run_id, self.config.launch_files, is_core=False)
        self.launch.start()


    def spin(self):
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        rospy.logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            while not rospy.core.is_shutdown() and not self.is_finished:
                self.launch.spin_once()
                rospy.rostime.wallsleep(0.01)
        except KeyboardInterrupt:
            rospy.logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')
        else:
            self.shutdown()
            rospy.core.signal_shutdown('timeout')


    def set_parameters(self, parameters):
        for key, value in parameters.items():
            key = "/{}/hri/{}".format(self.config.robot_id, key)
            rospy.loginfo("Set parameter: {}:{}".format(key, value))
            rospy.set_param(key, value)


    def unset_parameters(self, parameters):
        for key in parameters.keys():
            key = "/{}/hri/{}".format(self.config.robot_id, key)
            rospy.delete_param(key)


    def setup(self):
        self.set_parameters(self.parameters)
        self.launch_services()
        self.clock_sub = rospy.Subscriber('/clock', Clock, self.clock_callback)
        # self.reset_simulation()

        # self.robot_pose_publisher.publish(self.robot_pose)


        # add config specific stuff to reasoning system, <- do this as defined by rosparameters inside the respective nodes instead

        # picker_pose_publisher.publish(self.picker_pose)



    def terminate_bags(self):
        for player in self.running_bags:
            player.terminate()
            player.join()
        self.running_bags = []


    def clock_callback(self, msg):
        if msg.clock.nsecs == 0:
            clock = msg.clock.to_sec()
            if clock >= self.config.termination_time + self.start_clock:
                rospy.loginfo("EXP: Timeout condition reached")
                self.is_finished = True
                return
            difference = clock - self.last_clock
            self.last_clock = clock
            # index = 0
            # while index < len(self.running_bags):
            #     if not self.running_bags[index] is None:
            #         del self.running_bags[i]
            #     else:
            #         index+=1
            try:
                while clock == self.config.get_next_behaviour_time() + self.start_clock:
                    behaviour = self.config.get_next_behaviour()
                    if behaviour["type"] == "rosbag":
                        rospy.loginfo("EXP: Playing rosbag action '{}'".format(behaviour["label"]))
                        player = Player(behaviour["filename"], topics=behaviour["topics"], start_time=behaviour["start"], duration=behaviour["duration"])
                        player.start()
                        # for topic, outmsg, t in bag.read_messages(topics=behaviour["topics"]):
                        #     secs = t.to_sec()
                        #     if secs >= start_time and secs <= end_time:
                        #         outmsg.header.stamp.secs += clock
                        #         # self.camera_publisher.publish(outmsg)
                        #         self.human_action_publisher.publish(outmsg)
                        self.running_bags.append(player)
                    elif behaviour["type"] == "message":
                        if behaviour["target"] == "picker_movement":
                            rospy.loginfo("EXP: Sending Movement message '{}'".format(behaviour["message"]))
                            self.picker_pose_publisher.publish(behaviour["message"])
                        elif behaviour["target"] == "action_label":
                            rospy.loginfo("EXP: Sending Behaviour message '{}'".format(behaviour["message"]))
                            outmsg = Action()
                            outmsg.action = behaviour["message"]
                            outmsg.id = rospy.get_param("{}/hri/target_picker".format(self.config.robot_id), "Picker02")
                            self.human_action_publisher.publish(outmsg)
            except AttributeError as err:
                rospy.logerr("EXP: clock_callback - {}".format(err))


    def shutdown(self):
        # self.terminate_bags()
        for bag in self.bags.values():
            bag.close()
        self.launch.shutdown()
        self.robco.cancel_movement()
        self.unset_parameters(self.parameters)
        rospy.loginfo("EXP: Experiment ended")
