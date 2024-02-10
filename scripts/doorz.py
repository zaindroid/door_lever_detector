#!/usr/bin/env python3
import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String  # Simple string message for demonstration

# Load a pre-trained object detection model. Adjust path as necessary.
model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/lever_detection/model/door.pt')
bridge = CvBridge()

def detect_lever_and_draw_boxes(image):
    # Detect objects in the image using the loaded model
    result = model(image, size=416)
    detection_results = result.pandas().xyxy[0]  # Extract detection results

    # Filter detection results for levers (assuming class '2' is for levers, adjust as necessary)
    lever_detections = detection_results[detection_results['class'] == 2]

    # Initialize an empty string to accumulate bounding box information
    detected_levers_str = ""

    # Process detected levers and prepare bounding box information
    for index, row in lever_detections.iterrows():
        xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
        confidence = row['confidence']
        detected_levers_str += f"Lever detected with confidence {confidence:.2f} at [{xmin}, {ymin}, {xmax}, {ymax}]\n"

    # Return the string representation of detected levers
    return detected_levers_str

def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)

    detected_levers_str = detect_lever_and_draw_boxes(cv_image)

    # Publish the detected levers
    if detected_levers_str:
        bbox_pub.publish(detected_levers_str)
    else:
        bbox_pub.publish("No levers detected")

if __name__ == '__main__':
    rospy.init_node('lever_detector_node')
    rospy.Subscriber("/image_raw", Image, image_callback)
    bbox_pub = rospy.Publisher("/detected_levers", String, queue_size=10)

    rospy.spin()
