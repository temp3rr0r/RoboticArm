# import numpy as np
# import cv2
# from matplotlib import pyplot as plt
#
# MIN_MATCH_COUNT = 10
#
# # img1 = cv2.imread('box.png',0)          # queryImage
# # img2 = cv2.imread('box_in_scene.png',0) # trainImage
# img1 = cv2.imread("picsQr/model2.png",0)          # queryImage
#
# capture_device = cv2.VideoCapture(0)  # TODO: Enable if want to use a local camera
# capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
# capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# capture_device.set(cv2.CAP_PROP_FPS, 15)
# _, img2 = capture_device.read()  # Capture frame-by-frame
# # img2 = cv2.imread('box_in_scene.png',0) # trainImage

from __future__ import print_function
import cv2 as cv
import numpy as np
import argparse
parser = argparse.ArgumentParser(description='Code for Feature Matching with FLANN tutorial.')
parser.add_argument('--input1', help='Path to input image 1.', default='box.png')
parser.add_argument('--input2', help='Path to input image 2.', default='box_in_scene.png')
args = parser.parse_args()

# img_object = cv.imread(cv.samples.findFile(args.input1), cv.IMREAD_GRAYSCALE)
img_object = cv.imread(cv.samples.findFile("picsQr/model2.png"), cv.IMREAD_GRAYSCALE)

# img_scene = cv.imread(cv.samples.findFile(args.input2), cv.IMREAD_GRAYSCALE)
capture_device = cv.VideoCapture(0)  # TODO: Enable if want to use a local camera
capture_device.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
capture_device.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
capture_device.set(cv.CAP_PROP_FPS, 15)
_, video_frame = capture_device.read()  # Capture frame-by-frame
img_scene = cv.cvtColor(video_frame, cv.COLOR_BGR2GRAY)  # Convert images to gray-scale

if img_object is None or img_scene is None:
    print('Could not open or find the images!')
    exit(0)
#-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
minHessian = 400

# detector = cv.xfeatures2d_SURF.create(hessianThreshold=minHessian)
detector = cv.ORB_create(10)  # Detect ORB features and compute descriptors.

keypoints_obj, descriptors_obj = detector.detectAndCompute(img_object, None)
keypoints_scene, descriptors_scene = detector.detectAndCompute(img_scene, None)
#-- Step 2: Matching descriptor vectors with a FLANN based matcher
# Since SURF is a floating-point descriptor NORM_L2 is used
matcher = cv.DescriptorMatcher_create(cv.DescriptorMatcher_FLANNBASED)
knn_matches = matcher.knnMatch(descriptors_obj, descriptors_scene, 2)
#-- Filter matches using the Lowe's ratio test
ratio_thresh = 0.75
good_matches = []
for m,n in knn_matches:
    if m.distance < ratio_thresh * n.distance:
        good_matches.append(m)
#-- Draw matches
img_matches = np.empty((max(img_object.shape[0], img_scene.shape[0]), img_object.shape[1]+img_scene.shape[1], 3), dtype=np.uint8)
cv.drawMatches(img_object, keypoints_obj, img_scene, keypoints_scene, good_matches, img_matches, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
#-- Localize the object
obj = np.empty((len(good_matches),2), dtype=np.float32)
scene = np.empty((len(good_matches),2), dtype=np.float32)
for i in range(len(good_matches)):
    #-- Get the keypoints from the good matches
    obj[i,0] = keypoints_obj[good_matches[i].queryIdx].pt[0]
    obj[i,1] = keypoints_obj[good_matches[i].queryIdx].pt[1]
    scene[i,0] = keypoints_scene[good_matches[i].trainIdx].pt[0]
    scene[i,1] = keypoints_scene[good_matches[i].trainIdx].pt[1]
H, _ =  cv.findHomography(obj, scene, cv.RANSAC)
#-- Get the corners from the image_1 ( the object to be "detected" )
obj_corners = np.empty((4,1,2), dtype=np.float32)
obj_corners[0,0,0] = 0
obj_corners[0,0,1] = 0
obj_corners[1,0,0] = img_object.shape[1]
obj_corners[1,0,1] = 0
obj_corners[2,0,0] = img_object.shape[1]
obj_corners[2,0,1] = img_object.shape[0]
obj_corners[3,0,0] = 0
obj_corners[3,0,1] = img_object.shape[0]
scene_corners = cv.perspectiveTransform(obj_corners, H)
#-- Draw lines between the corners (the mapped object in the scene - image_2 )
cv.line(img_matches, (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])),\
    (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[1,0,0] + img_object.shape[1]), int(scene_corners[1,0,1])),\
    (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[2,0,0] + img_object.shape[1]), int(scene_corners[2,0,1])),\
    (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])), (0,255,0), 4)
cv.line(img_matches, (int(scene_corners[3,0,0] + img_object.shape[1]), int(scene_corners[3,0,1])),\
    (int(scene_corners[0,0,0] + img_object.shape[1]), int(scene_corners[0,0,1])), (0,255,0), 4)
#-- Show detected matches
cv.imshow('Good Matches & Object detection', img_matches)
cv.waitKey()