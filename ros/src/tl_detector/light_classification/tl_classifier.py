import os 
import rospy
import yaml
import tensorflow as tf
import numpy as np
from styx_msgs.msg import TrafficLight

curr_working_dir = os.path.dirname(os.path.realpath(__file__))

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        os.chdir(curr_working_dir)
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.is_site = self.config["is_site"]
        MODEL_FILE_NAME = self.config['model_file_name']

        if(self.is_site):
            print("-----Testing on site-----------------")
        else:
            print("----------Testing on simulator--------")

        print("Loaded model file name is :-",MODEL_FILE_NAME)

         # setup tensorflow graph
        self.graph = tf.Graph()


        with self.graph.as_default():
            od_graph = tf.GraphDef()
            with tf.gfile.GFile(MODEL_FILE_NAME,'rb') as fid:
                serialized_graph = fid.read()
                od_graph.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph, name='') 
            self.session = tf.Session(graph=self.graph)
            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        with self.graph.as_default():
            exp_img = np.expand_dims(image, axis=0)
            (scores, classes) = self.session.run([self.scores, self.classes], feed_dict={self.image_tensor: exp_img})

        classes = list(np.squeeze(classes))
        scores = np.squeeze(scores)

        if len(scores) == 0 or scores[0] < 0.1:
            return TrafficLight.UNKNOWN
        else:
            result_class = classes[0]
            if result_class == 2:
                return TrafficLight.RED
            elif result_class == 1:
                return TrafficLight.GREEN
            elif result_class == 3:
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
