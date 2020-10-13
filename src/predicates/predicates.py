
from opencog.atomspace import TruthValue


def deduction_formula(AC, AB, BC):
    tv1 = AB.tv
    tv2 = BC.tv
    if tv1.mean > 0.5 and tv2.mean > 0.5 and tv1.confidence > 0.5 and tv2.confidence > 0.5:
        AC.tv = TruthValue(1, 1)
    else:
        AC.tv = TruthValue(0, 0)
    return AC


def _float_value2object_state(float_value, name):
    l = float_value.to_list()
    if name.startsWith("thorvald_"):
        return Object_State(name=name, timestamp=l[2], x=l[0], y=l[1], xsize=ROBOT_WIDTH, ysize=ROBOT_LENGTH, object_type="Person")
    else:
        return Object_State(name=name, timestamp=l[2], x=l[0], y=l[1], xsize=PICKER_WIDTH, ysize=PICKER_LENGTH, object_type="Person")


def direction_formula(A, B):
    world = World_Trace()
    world.add_object_state_series(map(lambda fl: _float_value2object_state(fl, A.name), A.get_value(PredicateNode("position")).to_list()))
    world.add_object_state_series(map(lambda fl: _float_value2object_state(fl, B.name), B.get_value(PredicateNode("position")).to_list()))

    dynamic_args = {"qtcbs": {"quantisation_factor": 0.1,
                          "validate": True,
                          "no_collapse": True,
                          "qsrs_for": [(A.name, B.name)]}}
    qsrlib_request_message = QSRlib_Request_Message(which_qsr="qtcbs", input_data=world, dynamic_args=dynamic_args)
    try:
        qsrlib_response_message = self.qsrlib.request_qsrs(qsrlib_request_message)
        t = qsrlib_response_message.qsrs.get_sorted_timestamps()[-1]
        for k, v in qsrlib_response_message.qsrs.trace[t].qsrs.items():
            picker = k.split(",")[1]
            direction = v.qsr.get("qtcbs").split(",")[1]
            self.directions[picker] = direction
            update_direction = False
            try:
                update_direction = self.latest_directions[picker] != direction
            except:
                update_direction = True
                self.latest_directions[picker] = direction
            if update_direction:
                if direction == "+":
                    self.world_state.leaving(self.kb.concept(picker)).tv = self.kb.TRUE
                    rospy.logwarn("BDI: {} is leaving".format(picker))
                elif direction == "-":
                    self.world_state.approaching(self.kb.concept(picker)).tv = self.kb.TRUE
                    rospy.logwarn("BDI: {} is approaching".format(picker))
                else:
                    self.world_state.standing(self.kb.concept(picker)).tv = self.kb.TRUE
                    rospy.logwarn("BDI: {} is standing".format(picker))
                self.latest_directions[picker] = direction
