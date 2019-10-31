
forward = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 90.0" }
backward = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 90.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }
rotate_right = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 180.0" }
rotate_left = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }

stop = {"Left:Shoulder-X": "a = 270.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 270.0", "Right:Elbow-X": "a = 180.0" }
right_arm_up = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 270.0", "Right:Elbow-X": "a = 180.0" }

picking_berries_left = {"Left:Elbow-X": "p < Left:Hip-X", "Left:Ankle-Y": "p = Right:Ankle-Y", "Left:Ankle-X": "p = Right:Ankle-X", "Left:Hip-X": "p = Right:Hip-X"}

picking_berries_right = {"Right:Elbow-X": "p > Right:Hip-X", "Left:Ankle-Y": "p = Right:Ankle-Y", "Left:Ankle-X": "p = Right:Ankle-X", "Left:Hip-X": "p = Right:Hip-X"}

has_crate = {"Left:Elbow-X": "p = Left:Shoulder-X", "Right:Elbow-X": "p = Right:Shoulder-X", "Right:Wrist-X": "p < Right:Shoulder-X", "Left:Wrist-X": "p > Left:Shoulder-X", "Left:Wrist-Y": "p > Left:Shoulder-Y", "Right:Wrist-Y": "p > Right:Shoulder-Y", "Left:Elbow-Y": "p > Left:Shoulder-Y", "Right:Elbow-Y": "p > Right:Shoulder-Y"}

standing = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0", "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0", "Left:Ankle-Y": "p = Right:Ankle-Y"}




facing_towards = {"Right:Shoulder-X": "p > Left:Shoulder-X"}
facing_away = {"Right:Shoulder-X": "p < Left:Shoulder-X"}

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

pose_list = {"request service": right_arm_up,
             "picking berries left": picking_berries_left,
             "picking berries right": picking_berries_right,
             "come": forward,
             "go": backward,
             "standing": standing}
