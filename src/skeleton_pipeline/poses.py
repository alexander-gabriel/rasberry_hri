
forward = {"Left:Shoulder-X": 180.0, "Left:Elbow-X" : 90.0,  "Right:Shoulder-X": 90.0, "Right:Elbow-X" : 180.0 }
backward = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": 180.0, "Right:Elbow-X" : 90.0 }
stop = {"Left:Shoulder-X": -90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": -90.0, "Right:Elbow-X" : 180.0 }
come = {"Left:Shoulder-X": 90.0, "Left:Elbow-X" : 180.0,  "Right:Shoulder-X": -90.0, "Right:Elbow-X" : 180.0 }


pose_list = {"move forward": forward,
             "move backward": backward,
             "cancel": stop,
             "hail": come}
