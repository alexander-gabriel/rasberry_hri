
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace

import actionlib
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from rasberry_people_perception.topological_localiser import TopologicalNavLoc

from bdi_system import BDISystem

class Scheduler:

    def __init__(self, robot_name):


        self.robot_name = robot_name
        self.robot_control = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        self.robot_control.wait_for_server()

        self.qsrlib = QSRlib()
        self.options = sorted(self.qsrlib.qsrs_registry.keys())
        self.which_qsr = "rcc8"#"tpcc"

        self.locator = TopologicalNavLoc()
        for node in self.locator.tmap.nodes:
            self.bdi.world_state.db.add("is_a_place({:})".format(node.name))

        self.bdi = BDISystem()



        # --- begin tests setup ---
        self.bdi.world_state.add_predicate("is_a_place(place)")
        self.bdi.world_state.add_predicate("is_at(thing, place)")
        self.bdi.world_state.add_predicate("is_near(thing, place)")
        self.bdi.world_state.add_predicate("has_berries(place)")
        self.bdi.world_state.add_formula("in_front_of(?x, ?y) => !in_front_of(?y, ?x)", INF)
        self.bdi.world_state.add_formula("has_crate(?x) => !has_requested_crate(?x)", INF)
        # bself.di.world_state.db.retract("has_requested_crate(Peter)")
        self.bdi.world_state.db.add("has_requested_crate(Peter)")

        # self.bdi.world_state.db.retract("!has_crate(Peter)")
        # self.bdi.world_state.db.add("has_crate(Peter)")

        # self.bdi.world_state.db.retract("has_crate(Thorvald019)")
        self.bdi.world_state.db.add("has_crate(Thorvald019)")

        # self.bdi.world_state.db.retract("!in_front_of(Peter,Thorvald019)")
        self.bdi.world_state.db.add("!in_front_of(Peter,Thorvald019)")

        self.bdi.world_state.db.add("can_reach(Thorvald019,Peter)")

        # self.bdi.world_state.add_formula("is_a(Peter, Human)", 1.0)
        # self.bdi.write()
        # print(self.bdi.world_state.query.mln.predicate("in_front_of"))

        # --- end tests setup ---

        setup(self.bdi, self.robot_name)
        self.robot_sub = rospy.Subscriber('/robot_pose', Pose, self.robot_position_coordinate_callback)
        self.robot_sub = rospy.Subscriber('/current_node', String, self.robot_position_node_callback)
        self.people_sub = rospy.Subscriber("/picker%02d/posestamped", PoseStamped, self.people_tracker_callback)
        rospy.Subscriber("/lcas/hri/poses", Joints, self.human_intention_callback)

        # --- tests ---
        self.bdi.world_state.db.write()
        self.bdi.loop()
        self.bdi.world_state.db.write()
        self.bdi.loop()
        self.bdi.world_state.db.write()
        self.bdi.loop()
        self.bdi.world_state.db.write()


    def spin():
        """
        Blocks until ROS node is shutdown. Yields activity to other threads.
        @raise ROSInitException: if node is not in a properly initialized state
        """

        if not rospy.core.is_initialized():
            raise rospy.exceptions.ROSInitException("client code must call rospy.init_node() first")
        logdebug("node[%s, %s] entering spin(), pid[%s]", rospy.core.get_caller_id(), rospy.core.get_node_uri(), os.getpid())
        try:
            while not rospy.core.is_shutdown():
                loop()
                rospy.rostime.wallsleep(0.5)
        except KeyboardInterrupt:
            logdebug("keyboard interrupt, shutting down")
            rospy.core.signal_shutdown('keyboard interrupt')


    def robot_position_coordinate_callback(self, msg):
        self.bdi.robot_track.append(Object_State(name=id, timestamp=msg.header.time, x=msg.position.x, y=msg.position.y, width=1.35584, length=1.5))


    def robot_position_node_callback(self, msg):
        self.bdi.world_state.db.add("is_at({:},{:})".format(self.robot_name, msg))


    def human_intention_callback(self, msg):
        # TODO: match detected person to symbol
        if msg.pose == "request service":
            self.bdi.world_state.db.add("has_requested_crate(Peter)")


    def people_tracker_callback(self, msg, id):
        #TODO write lambda wrapper for callback to send id as well
        if not id in self.bdi.people_tracks:
            self.bdi.people_tracks[id] = []
        self.bdi.people_tracks[id].append(Object_State(name=id, timestamp=msg.header.time, x=msg.pose.position.x, y=msg.pose.position.y, width=0.6, length=0.4))
        (current_node, closest_node) = self.locator.localise_pose(msg)
        if current_node != "none":
            self.bdi.world_state.db.add("is_at({:},{:})".format(id, current_node))
        else:
            self.bdi.world_state.db.add("is_near({:},{:})".format(id, closest_node))



    def move_to(self, goal):
        print("Requesting Navigation to {:}".format(goal))
        navgoal = GotoNodeGoal()
        navgoal.target = goal
        #navgoal.origin = orig

        # Sends the goal to the action server.
        self.robot_control.send_goal(navgoal)#,self.done_cb, self.active_cb, self.feedback_cb)

        # Waits for the server to finish performing the action.
        self.robot_control.wait_for_result()

        # Prints out the result of executing the action
        ps = self.robot_control.get_result()  # A FibonacciResult
        print(ps)


    def cancel_movement(self):
        self.robot_control.cancel_all_goals()


    def save(self):
        self.bdi.save()
