
forward = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 180.0, "Right:Elbow-X" : 90.0 }
backward = {"Left:Shoulder-X": 180.0, "Left:Elbow-X" : 90.0,  "Right:Shoulder-X": 90.0, "Right:Elbow-X" : 180.0 }
rotate_right = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 180.0, "Right:Elbow-X" : 180.0 }
rotate_left = {"Left:Shoulder-X": 180.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 90.0, "Right:Elbow-X" : 180.0 }



stop = {"Left:Shoulder-X": 270.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 270.0, "Right:Elbow-X" : 180.0 }
right_arm_up = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 270.0, "Right:Elbow-X" : 180.0 }


pose_list = {"move forward": forward,
             "move backward": backward,
             "move left": rotate_right,
             "move right": rotate_left,
             "cancel": stop,
             "request service": right_arm_up}
