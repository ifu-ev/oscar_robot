#!/usr/bin/env python
import rospy # Python library for ROS
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 as cv # OpenCV library
import numpy as np # NumPy

def increase_brightness(img, value=100):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv.merge((h, s, v))
    img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return img

def detectAndDisplay(frame, face_cascade, visualize):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)
    height, width = frame_gray.shape
    #-- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x,y,w,h) in faces:
        center = (x + w//2, y + h//2)
        center_normalized = (float(center[0]) / width, float(center[1]) / height)
        frame = cv.ellipse(frame, center, (w//2, h//2), 0, 0, 360, (255, 0, 255), 4)
    if visualize:    
        cv.imshow('Capture - Face detection', frame)   
        cv.waitKey(3)
    if len(faces) > 0:
        return center_normalized
    else:
        return 0, 0

def publish_message():
    rospy.init_node("facedetection_node")

    pub = rospy.Publisher("/facedetection", Point, queue_size=1)
    pub_img = rospy.Publisher("/facedetection_image/compressed", CompressedImage, queue_size=1)

    face_cascade = cv.CascadeClassifier(rospy.get_param("~face_cascade", ""))
    camera_device = rospy.get_param("~camera_device", 0) # default laptop webcam
    visualize = rospy.get_param("~visualize", default=False)
    add_brightness = rospy.get_param("~add_brightness", default=0)

    bridge = CvBridge()

    #-- 2. Read the video stream
    cap = cv.VideoCapture(camera_device)
    if not cap.isOpened:
        rospy.loginfo('--(!)Error opening video capture')
        exit(0)
    #Set the resolution
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # height, width, channel = np.shape(frame)
        # frame = frame[:, 0:int(width/2)]
        if frame is None:
            rospy.loginfo('--(!) No captured frame -- Break!')
            break
        frame = increase_brightness(frame, add_brightness)
        x, y = detectAndDisplay(frame, face_cascade, visualize)
        point_msg = Point(x=x, y=y, z=0)
        image_msg = bridge.cv2_to_compressed_imgmsg(frame)
        pub_img.publish(image_msg)
        pub.publish(point_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_message()
    except rospy.ROSInterruptException:
        pass
