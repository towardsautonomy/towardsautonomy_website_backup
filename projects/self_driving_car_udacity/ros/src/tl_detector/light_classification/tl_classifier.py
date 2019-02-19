from __future__ import division

import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self, simulator_mode = True):
        # Initialize YOLO detection parameters
        self.detection_threshold = 0.7
        self.in_width = 416
        self.in_height = 416
        self.scale = 1/255
        self.num_classes = 3 # Classes: RED = 0, YELLOW = 1, GREEN = 2

        model_config = '/capstone/ros/src/tl_detector/light_classification/traffic_lights-yolov2-tiny.cfg'
        model_weights = '/capstone/ros/src/tl_detector/light_classification/traffic_lights_model-{}.weights'
        if simulator_mode:
            model_weights = model_weights.format('simulator')
        else:
            model_weights = model_weights.format('site')

        self.net = cv2.dnn.readNetFromDarknet(model_config, model_weights)

    # Get the names of the output layers
    def get_output_layers(self):
        # Get the names of all the layers in the network
        layersNames = self.net.getLayerNames()
        # Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layersNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Create input blob and send it to the network
        blob = cv2.dnn.blobFromImage(image, self.scale,
            (self.in_width, self.in_height), (0,0,0), True, crop=False)
        self.net.setInput(blob)

        # Run inference through the network
        outs = self.net.forward(self.get_output_layers())

        # Gather predictions from output layers
        out_classes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                # Filter out predictions below threshold
                if confidence > self.detection_threshold:
                    out_classes.append(classId)

        state = TrafficLight.UNKNOWN
        # In case of multiple traffic lights are detected in an image,
        # ensure a majority of the detected states match
        # Order of preference in case of more than 1 states detected equally:
        # Red, Yellow, Green
        if len(out_classes) > 0:
            state = np.argmax(np.bincount(out_classes))

        return state
