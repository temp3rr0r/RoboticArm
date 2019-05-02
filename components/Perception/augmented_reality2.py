from __future__ import print_function
import cv2
import numpy as np
import sys
# from getCosmosDbData import getLastDbDocument

MAX_FEATURES = 900
MIN_MATCHES = 30 # 15
GOOD_MATCH_PERCENT = 0.3
FLASH_EVERY_FRAMES = 40.0
MIN_DESCRIPTOR_DISTANCE_SUM = 10000

def plotGraph( vals, yRange):

    rangeIterations = range(min(vals), max(vals))
    scale = 1.0 / np.ceil(rangeIterations[1] - rangeIterations[0])
    bias = rangeIterations[0]
    rows = yRange[1] - yRange[0] + 1
    image = np.zeros((rows, len(vals)), dtype=int)
    for i in range(0, len(vals) - 1):
        cv2.line(image, (i, int(rows - 1 - (vals[i] - bias) * scale * yRange[1])), (i + 1, int(rows - 1 - (vals[i + 1] - bias) * scale * yRange[1])), (255, 0, 0), 5, cv2.LINE_AA)
    return image


def alignImages(videoFrame, modelReference, lastDocumentDbData, flashFrame):
    flashLogoWeightRatio = flashFrame / float(FLASH_EVERY_FRAMES)

    videoFrameGray = cv2.cvtColor(videoFrame, cv2.COLOR_BGR2GRAY)  # Convert images to gray-scale
    videoFrameGray = cv2.medianBlur(videoFrameGray, 3)

    orbFeatures = cv2.ORB_create(MAX_FEATURES)  # Detect ORB features and compute descriptors.
    keypoints1, descriptors1 = orbFeatures.detectAndCompute(videoFrameGray, None)
    descriptorMatcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)  # Match features

    modelReferences = ["picsQr/model0.png","picsQr/model1.png", "picsQr/model2.png",
                       "picsQr/model3.png", "picsQr/model4.png", "picsQr/model5.png"]

    mintotalDescriptorDistance = sys.maxsize
    detectedModel = -1

    for i in range(len(modelReferences)):
        current_modelReference = cv2.imread(modelReferences[i], cv2.IMREAD_COLOR)  # Reference Image
        modelReferenceGray = cv2.cvtColor(current_modelReference, cv2.COLOR_BGR2GRAY)
        modelReferenceGray = cv2.medianBlur(modelReferenceGray, 3)

        current_keypoints2, descriptors2 = orbFeatures.detectAndCompute(modelReferenceGray, None)
        current_descriptorMatches = descriptorMatcher.match(descriptors1, descriptors2, None)
        current_descriptorMatches.sort(key=lambda x: x.distance, reverse=False)  # Sort matches by score
        numGoodMatches = int(len(current_descriptorMatches) * GOOD_MATCH_PERCENT)  # Remove mediocre matches
        current_descriptorMatches = current_descriptorMatches[:numGoodMatches]
        totalDescriptorDistance = 0
        for x in current_descriptorMatches:
            totalDescriptorDistance += x.distance
        if totalDescriptorDistance < mintotalDescriptorDistance:
            mintotalDescriptorDistance = totalDescriptorDistance
            keypoints2 = current_keypoints2
            modelReference = current_modelReference
            descriptorMatches = current_descriptorMatches
            detectedModel = i

    if len(descriptorMatches) < MIN_MATCHES or mintotalDescriptorDistance < MIN_DESCRIPTOR_DISTANCE_SUM:
        return videoFrame  # Not good detection
    else:
        matchedPoints1 = np.zeros((len(descriptorMatches), 2), dtype=np.float32) # Extract location of good matches
        matchedPoints2 = np.zeros((len(descriptorMatches), 2), dtype=np.float32)

        for i, match in enumerate(descriptorMatches):
            matchedPoints1[i, :] = keypoints1[match.queryIdx].pt
            matchedPoints2[i, :] = keypoints2[match.trainIdx].pt

        height, width, _ = modelReference.shape
        homography, _ = cv2.findHomography(matchedPoints2, matchedPoints1, cv2.RANSAC)  # Find homography
        rectanglePoints = np.float32([[0, 0], [0, height - 1], [width - 1, height - 1], [width - 1, 0]]).reshape(-1, 1, 2)
        transformedRectanglePoints = cv2.perspectiveTransform(rectanglePoints, homography)  # Use homography

        # Draw rectangle only if within a specific ROI %
        transformedRectanglePoints2 = np.int32(transformedRectanglePoints)
        minX = sys.maxsize
        maxX = 0
        minY = sys.maxsize
        maxY = 0
        for point in transformedRectanglePoints2:
            if minX > point[0][0]:
                minX = point[0][0]
            if maxX < point[0][0]:
                maxX = point[0][0]
            if minY > point[0][1]:
                minY = point[0][1]
            if maxY < point[0][1]:
                maxY = point[0][1]
        roiPercentMax = 0.1  # 0.25
        roiPercentMin = 0.05  # 0.1
        sideRatioMin = 0.6
        sideRatioMax = 1.2
        if maxX < (1 - roiPercentMax) * videoFrameGray.shape[1] and minX > roiPercentMax * videoFrameGray.shape[1] and maxY < (1 - roiPercentMax) * videoFrameGray.shape[0] and minY > roiPercentMax * videoFrameGray.shape[0]:
            # Do not draw if rectangle sides too small
            if abs(maxX - minX) > roiPercentMin * videoFrameGray.shape[1] and abs(maxY - minY) > roiPercentMin * videoFrameGray.shape[0]:
                if sideRatioMin <= (abs(maxX - minX) / float(abs(maxY - minY))) <= sideRatioMax: # TODO: do not draw if one side is > 2x the other sides
                    transformedRectangle = cv2.polylines(videoFrame, [np.int32(transformedRectanglePoints)], True,(0, 0, 255), 3,cv2.LINE_AA)  # Draw a rectangle that marks the found model in the frame

        # Blend with Canny
        # sigma = 0.33
        # v = np.median(videoFrame)  # Compute the median of the single channel pixel intensities
        # lower = int(max(0, (1.0 - sigma) * v))
        # upper = int(min(255, (1.0 + sigma) * v))
        # edges = cv2.Canny(videoFrame, lower, upper)  # Apply automatic Canny edge detection using the computed median
        # edgesGray = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # videoFrame = cv2.addWeighted(videoFrame, 1, edgesGray, 1.0, 0)

        text1 = ""
        text2 = ""
        if detectedModel == 0:
            text1 = 'Waterings: {} ({}mL)'.format(lastDocumentDbData['waterings'], lastDocumentDbData['totalWater'])
            text2 = 'Tank: {}mL'.format(4000 - lastDocumentDbData['totalWater'])
            classLogo = cv2.imread("picsQr/logo0.png", cv2.IMREAD_COLOR)
            textWarp1 = '{} times'.format(lastDocumentDbData['waterings'])  # Attach text on warped img
            textWarp2 = '{}mL'.format(lastDocumentDbData['totalWater'])
        if detectedModel == 1:
            text1 = 'Min Moisture: {}%'.format(round(lastDocumentDbData['minMoisture'], 2))
            text2 = 'Max Moisture: {}%'.format(round(lastDocumentDbData['maxMoisture'], 2))
            classLogo = cv2.imread("picsQr/logo1.png", cv2.IMREAD_COLOR)
            textWarp1 = '{}%-'.format(round(lastDocumentDbData['minMoisture'], 1))  # Attach text on warped img
            textWarp2 = '{}%'.format(round(lastDocumentDbData['maxMoisture'], 1))
        if detectedModel == 2:
            text1 = 'ESP32 LoRa'
            text2 = 'Solar Bank'
            classLogo = cv2.imread("picsQr/logo2.png", cv2.IMREAD_COLOR)
            textWarp1 = 'Solar'
            textWarp2 = 'Bank'
        if detectedModel == 3:
            text1 = 'Waterings: {} ({}mL)'.format(lastDocumentDbData['waterings'], lastDocumentDbData['totalWater'])
            text2 = 'Tank: {}mL'.format(4000 - lastDocumentDbData['totalWater'])
            classLogo = cv2.imread("picsQr/logo3.png", cv2.IMREAD_COLOR)
            textWarp1 = '{} times'.format(lastDocumentDbData['waterings'])  # Attach text on warped img
            textWarp2 = '{}mL'.format(round(lastDocumentDbData['totalWater'], 1))
        if detectedModel == 4:
            text1 = 'PID Controller:'
            text2 = 'P={} I={} D={}'.format(lastDocumentDbData['proportional'], lastDocumentDbData['integral'], lastDocumentDbData['derivative'])
            classLogo = cv2.imread("picsQr/logo4.png", cv2.IMREAD_COLOR)
            textWarp1 = 'PID P={} '.format(lastDocumentDbData['proportional'])
            textWarp2 = 'I={} D={}'.format(lastDocumentDbData['integral'], lastDocumentDbData['derivative'])
        if detectedModel == 5:
            text1 = 'PID setpoint min: {}%'.format(round(lastDocumentDbData['setPointMin'], 2))
            text2 = 'PID setpoint max: {}%'.format(round(lastDocumentDbData['setPointMax'], 2))
            classLogo = cv2.imread("picsQr/logo5.png", cv2.IMREAD_COLOR)
            textWarp1 = '{}%-'.format(round(lastDocumentDbData['setPointMin'], 1))  # Attach text on warped img
            textWarp2 = '{}%'.format(round(lastDocumentDbData['setPointMax'], 1))

        if maxX < (1 - roiPercentMax) * videoFrameGray.shape[1] and minX > roiPercentMax * videoFrameGray.shape[1] \
                and maxY < (1 - roiPercentMax) * videoFrameGray.shape[0] and minY > roiPercentMax * videoFrameGray.shape[0]:
            # Do not draw if rectangle sides too small
            if abs(maxX - minX) > roiPercentMin * videoFrameGray.shape[1] and abs(maxY - minY) > roiPercentMin *videoFrameGray.shape[0]:
                if sideRatioMin <= (abs(maxX - minX) / float(abs(maxY - minY))) <= sideRatioMax:  # TODO: do not draw if one side is > 2x the other sides
                    # Warp class logo perspective & flash alpha blend

                    cv2.putText(classLogo, textWarp1, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)  # Put text
                    cv2.putText(classLogo, textWarp2, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

                    emptyClassLogo = np.zeros((classLogo.shape[0],classLogo.shape[1],3), np.uint8)  # Create white shape logo
                    emptyClassLogo[:] = (255, 255, 255)

                    emptyClassLogoWarped = cv2.warpPerspective(emptyClassLogo, homography,(videoFrame.shape[1], videoFrame.shape[0]))  # Warp
                    classLogoWarped = cv2.warpPerspective(classLogo, homography, (videoFrame.shape[1], videoFrame.shape[0]))
                    emptyClassLogoGray = cv2.cvtColor(emptyClassLogoWarped, cv2.COLOR_BGR2GRAY)

                    ret, mask = cv2.threshold(emptyClassLogoGray, 10, 255, cv2.THRESH_BINARY)  # Create masks
                    mask_inv = cv2.bitwise_not(mask)
                    videoFrameBackground = cv2.bitwise_and(videoFrame, videoFrame, mask=mask_inv)
                    videoFrameForeground = cv2.bitwise_and(videoFrame, videoFrame, mask=mask)

                    classLogoWarped = cv2.addWeighted(classLogoWarped, 1 - flashLogoWeightRatio, videoFrameForeground, flashLogoWeightRatio, 0)  # Blend & flash
                    videoFrame = cv2.add(classLogoWarped, videoFrameBackground)


        # Put text on top left corner
        cv2.putText(videoFrame, text1, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(videoFrame, text2, (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 2, cv2.LINE_AA)
        text4 = "Feature matches (ORB): {}".format(len(descriptorMatches))
        cv2.putText(videoFrame, text4, (50, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        text3 = "Hamming distance (sum): {}".format(int(mintotalDescriptorDistance))
        cv2.putText(videoFrame, text3, (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        text4 = "Class: {}".format(detectedModel)
        cv2.putText(videoFrame, text4, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)

        # TODO: plot graph of last 5 values?
        return videoFrame


if __name__ == '__main__':

    writeVideo = False

    modelReference = cv2.imread("picsQr/model0.png", cv2.IMREAD_COLOR)

    captureDevice = cv2.VideoCapture(0)  # TODO: Enable if want to use a local camera
    # captureDevice = cv2.VideoCapture('picsQr/vids/good8.mp4')  # TODO: Enable if u want to use a video file


    # TODO: set resolution

    captureDevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    captureDevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    captureDevice.set(cv2.CAP_PROP_FPS, 15)

    # lastDocumentDbData = getLastDbDocument()  # Get last document db data
    lastDocumentDbData = {'setPointMin': 0, 'setPointMax': 1, 'maxMoisture': 80,
                          'totalWater': 3000, 'waterings': 3, 'minMoisture': 30,
                          'proportional': 1.2, 'integral': 0.8, 'derivative': 1.0}  # TODO: dummy data

    if writeVideo:
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('good8.avi', fourcc, 20, (1920, 1080))

    flashFrame = 0

    while (True):
        _, videoFrame = captureDevice.read()  # Capture frame-by-frame

        # print("videoFrame.shape", videoFrame.shape)
        # break

        alignedFrame = alignImages(videoFrame, modelReference, lastDocumentDbData, flashFrame)
        flashFrame += 1
        if flashFrame >= FLASH_EVERY_FRAMES * 0.75:
            flashFrame = 0
        cv2.imshow('Video Frame', alignedFrame)  # Display the resulting frame

        if writeVideo:
            out.write(alignedFrame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

captureDevice.release()  # When everything done, release the capture
if writeVideo:
    out.release()
cv2.destroyAllWindows()
