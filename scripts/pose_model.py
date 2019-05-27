#!/usr/bin/env python
import sys

import rospy
from sensor_msgs.msg import Image
from image_recognition_msgs.srv import Recognize

import tensorflow as tf

class PoseModel:
    def __init__(self):
        rospy.init_node('pose_model', anonymous=False)
        self.graph = tf.Graph()

    def callback(self, data):
        try:
            if (self.service.died):
                rospy.signal_shutdown("openpose not found")
            if (self.count < 100000):
                if not self.service.isAlive():
                    self.service = Openpose(self.interface)
                    self.count = self.count + 1
                    self.service.data = data
                    self.service.start()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

if __name__ == '__main__':
    rospy.myargv(argv=sys.argv)
    se = PoseModel()
    rospy.spin()


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

    graph = tf.Graph()
    with graph.as_default():
        # Make sure you allow_soft_placement, some ops have to be put on the CPU (e.g. summary operations)
        session = tf.Session(config=tf.ConfigProto(
            allow_soft_placement=True,
            log_device_placement=False))

        num_inputs = 1024
        dims = 10
        clusters = 3
        # Makes toy clusters with pretty clear separation, see the sklearn site for more info
        blob_data = make_blobs(num_inputs, dims, clusters)
        # Scale the blob data for easier training. Also index 0 because the output is a (data, label) tuple.
        scaler = StandardScaler()
        input_data = scaler.fit_transform(blob_data[0])
        batch_size = 128

        # Build the TensorFlow dataset pipeline per the standard tutorial.
        dataset = tf.data.Dataset.from_tensor_slices(input_data.astype(np.float32))
        dataset = dataset.repeat()
        dataset = dataset.batch(batch_size)
        iterator = dataset.make_one_shot_iterator()
        next_element = iterator.get_next()

        # This is more neurons than you need but it makes the visualization look nicer
        m = 20
        n = 20

        # Build the SOM object and place all of its ops on the graph
        som = SelfOrganizingMap(m=m, n=n, dim=dims, max_epochs=20, gpus=1, session=session, graph=graph,
                                input_tensor=next_element, batch_size=batch_size, initial_learning_rate=0.1)

        init_op = tf.global_variables_initializer()
        session.run([init_op])

        # Note that I don't pass a SummaryWriter because I don't really want to record summaries in this script
        # If you want Tensorboard support just make a new SummaryWriter and pass it to this method
        som.train(num_inputs=num_inputs)

        weights = som.output_weights

        umatrix = get_umatrix(weights, m, n)
        fig = plt.figure()
        plt.imshow(umatrix, origin='lower')
        plt.show(block=True)
