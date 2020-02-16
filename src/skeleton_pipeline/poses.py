

# 0,0 is left, top
forward = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 90.0" }
backward = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 90.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }
rotate_right = {"Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 180.0", "Right:Elbow-X": "a = 180.0" }
rotate_left = {"Left:Shoulder-X": "a = 180.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 90.0", "Right:Elbow-X": "a = 180.0" }

stop = {"Left:Shoulder-X": "a = 270.0", "Left:Elbow-X": "a = 180.0",  "Right:Shoulder-X": "a = 270.0", "Right:Elbow-X": "a = 180.0" }
right_arm_up = {
    "Left:Shoulder-X": "a = 90.0", "Left:Elbow-X": "a = 180.0", # left arm down and straight
    "Right:Shoulder-X": "a > 230.0", "Right:Elbow-X": "a = 180.0" #right arm up and straight
    }

picking_berries_left = {
    "Left:Elbow-X": "p < Left:Hip-X", "Right:Elbow-X": "p < Left:Hip-X", #arms to the left
    "Left:Ankle-Y": "p = Right:Ankle-Y", "Left:Ankle-X": "p = Right:Ankle-X", #feet together behind one another
    "Left:Hip-X": "p = Right:Hip-X", # hips behind one another
    "Left:Elbow-Y": "p > Left:Shoulder-Y", "Right:Elbow-Y": "p > Left:Shoulder-Y" # elbows lower than shoulders
    }

picking_berries_right = {
    "Left:Elbow-X": "p > Right:Hip-X", "Right:Elbow-X": "p > Right:Hip-X", #arms to the right
    "Left:Ankle-Y": "p = Right:Ankle-Y", "Left:Ankle-X": "p = Right:Ankle-X", #feet together behind one another
    "Left:Hip-X": "p = Right:Hip-X", # hips behind one another
    "Left:Elbow-Y": "p > Right:Shoulder-Y", "Right:Elbow-Y": "p > Right:Shoulder-Y" # elbows lower than shoulders
    }

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

pose_list = {"call robot": right_arm_up,
             "picking berries left": picking_berries_left,
             "picking berries right": picking_berries_right,
             "standing": standing}
