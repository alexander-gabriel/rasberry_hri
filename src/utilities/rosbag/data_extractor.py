

def get_rosbag_from_file(filename, mode='r'):
    try:
        bag = rosbag.Bag(filename, mode)
        return bag
    except Exception as e:
        print("Failed to get rosbag topics info from file {:} with exception: '{:}'".format(filename, e))
        return None


class Player():

    def __init__(self, filename, topics=[], start_time=None):
        super(Player, self).__init__()
        self.bag = get_rosbag_from_file(filename)
        self.msgs = self.bag.read_messages(topics=topics)
        self.pubs = {}
        self.start_timestamp = start_time


    def run(self):
        self.category_counts = {}
        count = 0
        for topic, msg, timestamp in self.msgs:
            try:
                self.category_counts[msg.action] += 1
            except:
                self.category_counts[msg.action] = 1
            count +=1


    def close(self):
        self.bag.close()








if __name__ == '__main__':
    rospy.init_node("play_record")
    ids = [1,2,3,4,5,6,7,8,9,10]
    sids = list()
    for id in ids:
        sids.append(str(id))
        sids.append("{}-moving".format(id))
    # sids = ["1"]
    # open database
    for sid in sids:
        ## TODO: readjust
        source_folder = os.path.join(PATH, "subject-{:}".format(sid))
        # source_folder = PATH
        target_folder = os.path.join(PATH, "subject-{:}-out".format(sid))
        try:
            os.mkdir(target_folder)
        except:
            pass
        print(source_folder)
        counts = {}
        for filename in os.listdir(source_folder):
            if filename.endswith(".bag"):
                try:
                    player = Player(os.path.join(source_folder,"{:}".format(filename)), topics=["/camera/color/image_raw"], start_time=None)
                    player.run()
                    try:
                        counts[filename.split("_")[0]].append(player.category_counts)
                    except:
                        counts[filename.split("_")[0]] = [player.category_counts]
