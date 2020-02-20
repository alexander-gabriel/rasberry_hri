
# 0,0 is left, top

front_facing = {
    "Left:Shoulder-X": "p > Neck-X",
    "Right:Shoulder-X": "p < Neck-X",
    "Left:Hip-X": "p > Neck-X",
    "Right:Hip-X": "p < Neck-X",
    # "Left:Knee-X": "p > Neck-X",
    # "Right:Knee-X" "p < Neck-X",
}

back_facing = {
    "Left:Shoulder-X": "p < Neck-X",
    "Right:Shoulder-X": "p > Neck-X",
    "Left:Hip-X": "p < Neck-X",
    "Right:Hip-X": "p > Neck-X",
    # "Left:Knee-X": "p < Neck-X",
    # "Right:Knee-X" "p > Neck-X",
}

side_facing = {
    "Left:Shoulder-X": "p = Neck-X",
    "Right:Shoulder-X": "p = Neck-X",
    "Left:Hip-X": "p = Neck-X",
    "Right:Hip-X": "p = Neck-X",
    # "Left:Knee-X": "p = Neck-X",
    # "Right:Knee-X" "p = Neck-X",
}

standing = {
    "Right:Knee-Y" : "p = Left:Knee-Y",
    "Right:Knee-Y" : "p = Left:Knee-Y",
}

right_arm_up = {"Right:Shoulder-X": "a > 230.0"}
right_arm_down = {"Right:Shoulder-X": "a = 90.0"}
right_arm_side = {"Right:Shoulder-X": "a = 180.0"}
right_arm_angled = {"Right:Elbow-X": "a = 90.0"}
right_arm_straight = {"Right:Elbow-X": "a = 180.0"}

left_arm_up = {"Left:Shoulder-X": "a > 230"}
left_arm_down = {"Left:Shoulder-X": "a = 90.0"}
left_arm_side = {"Left:Shoulder-X": "a = 180.0"}
left_arm_angled = {"Left:Elbow-X": "a = 90.0"}
left_arm_straight = {"Left:Elbow-X": "a = 180.0"}

arms_crossed_frontfacing = {
    "Left:Wrist-X": "p < Neck-X", "Right:Wrist-X": "p > Neck-X",
    "Left:Elbow-X": "p > Neck-X", "Right:Elbow-X": "p < Neck-X",
}

arms_side_frontfacing ={
    "Left:Wrist-X": "p > Neck-X", "Right:Wrist-X": "p < Neck-X",
    "Left:Elbow-X": "p > Neck-X", "Right:Elbow-X": "p < Neck-X",
}

arms_side_backfacing ={
    "Left:Wrist-X": "p < Neck-X", "Right:Wrist-X": "p > Neck-X",
    "Left:Elbow-X": "p < Neck-X", "Right:Elbow-X": "p > Neck-X",
}

arms_left ={
    "Left:Elbow-X": "p < Neck-X", #"Left:Wrist-X": "p < Left:Elbow-X",
    "Right:Elbow-X": "p < Neck-X", #"Right:Wrist-X": "p < Right:Elbow-X",
}

arms_right ={
    "Left:Elbow-X": "p > Neck-X", #"Left:Wrist-X": "p > Left:Elbow-X",
    "Right:Elbow-X": "p > Neck-X", #"Right:Wrist-X": "p > Right:Elbow-X",
}

arms_mid_high ={
    "Left:Wrist-Y": "p < Left:Hip-Y", "Right:Wrist-Y": "p < Right:Hip-Y",
    "Left:Wrist-Y": "p > Neck-Y", "Right:Wrist-Y": "p > Neck-Y",
    "Left:Elbow-Y": "p > Neck-Y", "Right:Elbow-Y": "p > Neck-Y",
}


gesture_call_robot = dict()
gesture_call_robot.update(front_facing)
gesture_call_robot.update(standing)
gesture_call_robot.update(right_arm_up)
gesture_call_robot.update(right_arm_straight)
gesture_call_robot.update(left_arm_down)
gesture_call_robot.update(left_arm_straight)

gesture_forward = dict()
gesture_forward.update(front_facing)
gesture_forward.update(standing)
gesture_forward.update(right_arm_side)
gesture_forward.update(right_arm_angled)
gesture_forward.update(left_arm_down)
gesture_forward.update(left_arm_straight)

gesture_backward = dict()
gesture_backward.update(front_facing)
gesture_backward.update(standing)
gesture_backward.update(left_arm_side)
gesture_backward.update(left_arm_angled)
gesture_backward.update(right_arm_down)
gesture_backward.update(right_arm_straight)

gesture_cancel = dict()
gesture_cancel.update(front_facing)
gesture_cancel.update(standing)
gesture_cancel.update(arms_crossed_frontfacing)
gesture_cancel.update(arms_mid_high)

gesture_stop = dict()
gesture_stop.update(front_facing)
gesture_stop.update(standing)
gesture_stop.update(arms_side_frontfacing)
gesture_stop.update(arms_mid_high)

put_or_get_crate = dict()
put_or_get_crate.update(back_facing)
put_or_get_crate.update(standing)
put_or_get_crate.update(arms_side_backfacing)
put_or_get_crate.update(arms_mid_high)

picking_berries_left = dict()
picking_berries_left.update(side_facing)
picking_berries_left.update(standing)
picking_berries_left.update(arms_left)
picking_berries_left.update(arms_mid_high)

picking_berries_right = dict()
picking_berries_right.update(side_facing)
picking_berries_right.update(standing)
picking_berries_right.update(arms_right)
picking_berries_right.update(arms_mid_high)


# rotate_right = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 180.0" }
# rotate_left = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }
# standing = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0", "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0", "Left:Ankle-Y": "p = Right:Ankle-Y"}

# facing_towards = {"Right:Shoulder-X": "p > Left:Shoulder-X"}
# facing_away = {"Right:Shoulder-X": "p < Left:Shoulder-X"}

# pose_list = {"move forward": forward,
#              "move backward": backward,
#              "move left": rotate_right,
#              "move right": rotate_left,
#              "cancel": stop,
#              "request service": right_arm_up,
#              "picking berries left": picking_berries_left,
#              "picking berries right": picking_berries_right,
#              "facing towards": facing_towards,
#              "facing away": facing_away,
#              "has cart": has_cart}

pose_list = {"call robot": gesture_call_robot,
             "picking berries left": picking_berries_left,
             "picking berries right": picking_berries_right,
             # "standing": standing,
             "cancel": gesture_cancel,
             "stop": gesture_stop,
             "put_or_get_crate": put_or_get_crate,
             "move forward": gesture_forward,
             "move backward": gesture_backward}
