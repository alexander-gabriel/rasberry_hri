
forward = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 180.0, "Right:Elbow-X" : 90.0 }
backward = {"Left:Shoulder-X": 180.0, "Left:Elbow-X" : 90.0,  "Right:Shoulder-X": 90.0, "Right:Elbow-X" : 180.0 }
turn_right = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 180.0, "Right:Elbow-X" : 180.0 }
turn_left = {"Left:Shoulder-X": 180.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 90.0, "Right:Elbow-X" : 180.0 }



stop = {"Left:Shoulder-X": -90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": -90.0, "Right:Elbow-X" : 180.0 }
come = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": -90.0, "Right:Elbow-X" : 180.0 }


pose_list = {"move forward": forward,
             "move backward": backward,
             "turn right": turn_right,
             "turn left": turn_left,
             "cancel": stop,
             "hail": come}
