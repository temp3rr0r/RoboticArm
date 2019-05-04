from __future__ import print_function
import cv2
import numpy as np
import sys

MAX_FEATURES = 900  # 900
MIN_MATCHES = 30  # 15
GOOD_MATCH_PERCENT = 0.3  # 0.3
FLASH_EVERY_FRAMES = 40.0
MIN_DESCRIPTOR_DISTANCE_SUM = 10000
use_flann = True

# TODO: load regressors
from sklearn.externals import joblib

# TODO: test predict to check loaded models
X_validate = [
    [1395.1582, 690.2671, 1404.3729, 840.06885, 1554.4364, 843.0976, 1544.1744, 692.78174],
    [425.08948, 688.82007, 400.65857, 837.42413, 562.5942, 840.0118, 563.503, 687.0966],
    [861.4465, 221.13101, 856.76025, 351.2697, 996.6109, 354.29453, 997.3974, 216.82195],
    [1216.8048, 108.742096, 1221.4166, 332.99548, 1443.5278, 328.23065, 1445.7577, 120.2978],
    [989.15936, 398.22433, 998.178, 576.02185, 1162.1074, 574.61816, 1170.0554, 406.4485]]
y_validate = [[-33.8, -17.8, 0.0],
              [-33.8, 17.8, 0.0],
              [-15.8, 2.2, 0.0],
              [-17.8, -7.8, 0.0],
              [-25.8, -2.8, 0.0]]
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_MLPRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_GradientBoostingRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_RandomForestRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_ExtraTreesRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_KNeighborsRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_AdaBoostRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_BaggingRegressor_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_LinearSVR_xyz.sav')
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_GaussianProcessRegressor_xyz.sav')  # TODO: good for xy
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_MultiOutputRegressor_xyz.sav')  # TODO: good for xy
second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_RANSACRegressor_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_RidgeCV_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_ElasticNet_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_MultiTaskElasticNetCV_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_Lasso_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_MultiTaskLasso_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_MultiTaskLassoCV_xyz.sav')  # TODO: good for xyz
# second_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_PLSRegression_xyz.sav')  # TODO: good for xyz
prediction_second_regressor = second_regressor_qr_to_arm_xyz.predict(X_validate)
print(prediction_second_regressor)
linear_regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_LinearRegressor_xyz.sav')  # X, Y ok
pred_linear_regressor = linear_regressor_qr_to_arm_xyz.predict(X_validate)
print(pred_linear_regressor)
print(y_validate)


def plot_graph(values, y_range):
    range_iterations = range(min(values), max(values))
    scale = 1.0 / np.ceil(range_iterations[1] - range_iterations[0])
    bias = range_iterations[0]
    rows = y_range[1] - y_range[0] + 1
    image = np.zeros((rows, len(values)), dtype=int)
    for i in range(0, len(values) - 1):
        cv2.line(image, (i, int(rows - 1 - (values[i] - bias) * scale * y_range[1])),
                 (i + 1, int(rows - 1 - (values[i + 1] - bias) * scale * y_range[1])), (255, 0, 0), 5, cv2.LINE_AA)
    return image


def align_images(video_frame, model_reference_in, last_data, flash_frame):
    flash_logo_weight_ratio = flash_frame / float(FLASH_EVERY_FRAMES)

    video_frame_gray = cv2.cvtColor(video_frame, cv2.COLOR_BGR2GRAY)  # Convert images to gray-scale
    # video_frame_gray = cv2.medianBlur(video_frame_gray, 3)

    orb_features = cv2.ORB_create(MAX_FEATURES)  # Detect ORB features and compute descriptors.
    keypoints1, descriptors1 = orb_features.detectAndCompute(video_frame_gray, None)

    if use_flann:
        FLANN_INDEX_KDTREE = 1
        FLANN_INDEX_LSH = 6
        search_params = {}
        # flann_params = dict(algorithm=FLANN_INDEX_LSH,
        #                     table_number=3,  # 12, 6
        #                     key_size=6,  # 20, 12
        #                     multi_probe_level=1)  # 2, 1
        flann_params = dict(algorithm=6)
        descriptor_matcher = cv2.FlannBasedMatcher(flann_params, search_params)  # bug : need to pass empty dict (#1329)
    else:
        descriptor_matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)  # Match features

    model_references = ["picsQr/model0.png", "picsQr/model1.png", "picsQr/model2.png", "picsQr/model3.png",
                        "picsQr/model4.png", "picsQr/model5.png"]

    min_total_descriptor_distance = sys.maxsize
    detected_model = -1

    for i in range(len(model_references)):
        current_model_reference = cv2.imread(model_references[i], cv2.IMREAD_COLOR)  # Reference Image
        model_reference_gray = cv2.cvtColor(current_model_reference, cv2.COLOR_BGR2GRAY)
        # model_reference_gray = cv2.medianBlur(model_reference_gray, 3)

        current_key_points_2, descriptors2 = orb_features.detectAndCompute(model_reference_gray, None)
        current_descriptor_matches = descriptor_matcher.match(descriptors1, descriptors2, None)
        # current_descriptor_matches = descriptor_matcher.knnMatch(descriptors1, descriptors2, 2)  # TODO: test knn

        current_descriptor_matches.sort(key=lambda x_point: x_point.distance, reverse=False)  # Sort matches by score
        num_good_matches = int(len(current_descriptor_matches) * GOOD_MATCH_PERCENT)  # Remove mediocre matches
        current_descriptor_matches = current_descriptor_matches[:num_good_matches]
        total_descriptor_distance = 0
        for x in current_descriptor_matches:
            total_descriptor_distance += x.distance
        if total_descriptor_distance < min_total_descriptor_distance:
            min_total_descriptor_distance = total_descriptor_distance
            keypoints_2 = current_key_points_2
            model_reference_in = current_model_reference
            descriptor_matches = current_descriptor_matches
            detected_model = i

    if len(descriptor_matches) < MIN_MATCHES or min_total_descriptor_distance < MIN_DESCRIPTOR_DISTANCE_SUM:
        return video_frame  # Not good detection
    else:
        matched_points1 = np.zeros((len(descriptor_matches), 2), dtype=np.float32)  # Extract location of good matches
        matched_points2 = np.zeros((len(descriptor_matches), 2), dtype=np.float32)

        for i, match in enumerate(descriptor_matches):
            matched_points1[i, :] = keypoints1[match.queryIdx].pt
            matched_points2[i, :] = keypoints_2[match.trainIdx].pt

        height, width, _ = model_reference_in.shape
        homography, _ = cv2.findHomography(matched_points2, matched_points1, cv2.RANSAC)  # Find homography
        rectangle_points = np.float32([[0, 0], [0, height - 1], [width - 1, height - 1], [width - 1, 0]]).reshape(-1, 1,
                                                                                                                  2)
        transformed_rectangle_points = cv2.perspectiveTransform(rectangle_points, homography)  # Use homography

        # TODO: position calibration seq:
        # TODO: 1. Request qr into points A-G
        # TODO: 1. Gather pixel points foreach position
        # TODO: 3. Train MPL or Linear regressor BBox 4 pixel points -> cm arm xyz
        # print("transformed_rectangle_points: {}".format(transformed_rectangle_points))

        # Draw rectangle only if within a specific ROI %
        transformed_rectangle_points2 = np.int32(transformed_rectangle_points)
        min_x = sys.maxsize
        max_x = 0
        min_y = sys.maxsize
        max_y = 0
        for point in transformed_rectangle_points2:
            if min_x > point[0][0]:
                min_x = point[0][0]
            if max_x < point[0][0]:
                max_x = point[0][0]
            if min_y > point[0][1]:
                min_y = point[0][1]
            if max_y < point[0][1]:
                max_y = point[0][1]
        roi_percent_max = 0.001 # 0.25
        roi_percent_min = 0.0002  # 0.1
        side_ratio_min = 0.6
        side_ratio_max = 1.2
        if max_x < (1 - roi_percent_max) * video_frame_gray.shape[1] and min_x > roi_percent_max * \
                video_frame_gray.shape[1] and max_y < (1 - roi_percent_max) * video_frame_gray.shape[
            0] and min_y > roi_percent_max * video_frame_gray.shape[0]:
            # Do not draw if rectangle sides too small
            if abs(max_x - min_x) > roi_percent_min * video_frame_gray.shape[1] and abs(
                    max_y - min_y) > roi_percent_min * video_frame_gray.shape[0]:
                if side_ratio_min <= (abs(max_x - min_x) / float(
                        abs(max_y - min_y))) <= side_ratio_max:  # TODO: do not draw if one side is > 2x the other sides
                    transformedRectangle = cv2.polylines(video_frame, [np.int32(transformed_rectangle_points)], True,
                                                         (0, 0, 255), 3,
                                                         cv2.LINE_AA)  # Draw a rectangle that marks the found model in the frame

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
        if detected_model == 0:
            text1 = 'Waterings: {} ({}mL)'.format(last_data['waterings'], last_data['totalWater'])
            text2 = 'Tank: {}mL'.format(4000 - last_data['totalWater'])
            class_logo = cv2.imread("picsQr/logo0.png", cv2.IMREAD_COLOR)
            text_warp1 = '{} times'.format(last_data['waterings'])  # Attach text on warped img
            text_warp2 = '{}mL'.format(last_data['totalWater'])
        if detected_model == 1:
            text1 = 'Min Moisture: {}%'.format(round(last_data['minMoisture'], 2))
            text2 = 'Max Moisture: {}%'.format(round(last_data['maxMoisture'], 2))
            class_logo = cv2.imread("picsQr/logo1.png", cv2.IMREAD_COLOR)
            text_warp1 = '{}%-'.format(round(last_data['minMoisture'], 2))  # Attach text on warped img
            text_warp2 = '{}%'.format(round(last_data['maxMoisture'], 2))
        if detected_model == 2:
            text1 = 'Item vs arm(Linear Regression): '
            text2 = '   (Multi-Output ElasticNet CV): '
            # TODO: do use regressors
            last_qr_position = [[transformed_rectangle_points[0][0][0], transformed_rectangle_points[0][0][1],
                                 transformed_rectangle_points[1][0][0], transformed_rectangle_points[1][0][1],
                                 transformed_rectangle_points[2][0][0], transformed_rectangle_points[2][0][1],
                                 transformed_rectangle_points[3][0][0], transformed_rectangle_points[3][0][1]]]
            # print("last_qr_position: {}".format(last_qr_position))
            # text1 = text1 + str(transformed_rectangle_points)
            arm_xyz_offset = [0.0, 0.0, 0.0]
            linear_regressor_predicted_arm_xyz = linear_regressor_qr_to_arm_xyz.predict(last_qr_position)
            mlp_regressor_predicted_arm_xyz = second_regressor_qr_to_arm_xyz.predict(last_qr_position)  # TODO: z position

            linear_regressor_predicted_arm_xyz[0] += arm_xyz_offset
            mlp_regressor_predicted_arm_xyz[0] += arm_xyz_offset

            linear_regressor_predicted_arm_xyz = np.round(linear_regressor_predicted_arm_xyz, 1)
            mlp_regressor_predicted_arm_xyz = np.round(mlp_regressor_predicted_arm_xyz, 1)

            text1 = text1 + "{} {} {}".format(linear_regressor_predicted_arm_xyz[0][0],
                                                         linear_regressor_predicted_arm_xyz[0][1],
                                                         linear_regressor_predicted_arm_xyz[0][2])
            text2 = text2 + "{} {} {}".format(mlp_regressor_predicted_arm_xyz[0][0],
                                                         mlp_regressor_predicted_arm_xyz[0][1],
                                                         mlp_regressor_predicted_arm_xyz[0][2])
            class_logo = cv2.imread("picsQr/logo2.png", cv2.IMREAD_COLOR)
            text_warp1 = 'Solar'
            text_warp2 = 'Bank'
        if detected_model == 3:
            text1 = 'Waterings: {} ({}mL)'.format(last_data['waterings'], last_data['totalWater'])
            text2 = 'Tank: {}mL'.format(4000 - last_data['totalWater'])
            class_logo = cv2.imread("picsQr/logo3.png", cv2.IMREAD_COLOR)
            text_warp1 = '{} times'.format(last_data['waterings'])  # Attach text on warped img
            text_warp2 = '{}mL'.format(round(last_data['totalWater'], 1))
        if detected_model == 4:
            text1 = 'PID Controller:'
            text2 = 'P={} I={} D={}'.format(last_data['proportional'], last_data['integral'], last_data['derivative'])
            class_logo = cv2.imread("picsQr/logo4.png", cv2.IMREAD_COLOR)
            text_warp1 = 'PID P={} '.format(last_data['proportional'])
            text_warp2 = 'I={} D={}'.format(last_data['integral'], last_data['derivative'])
        if detected_model == 5:
            text1 = 'PID setpoint min: {}%'.format(round(last_data['setPointMin'], 2))
            text2 = 'PID setpoint max: {}%'.format(round(last_data['setPointMax'], 2))
            class_logo = cv2.imread("picsQr/logo5.png", cv2.IMREAD_COLOR)
            text_warp1 = '{}%-'.format(round(last_data['setPointMin'], 1))  # Attach text on warped img
            text_warp2 = '{}%'.format(round(last_data['setPointMax'], 1))

        if max_x < (1 - roi_percent_max) * video_frame_gray.shape[1] and min_x > roi_percent_max * \
                video_frame_gray.shape[1] \
                and max_y < (1 - roi_percent_max) * video_frame_gray.shape[0] and min_y > roi_percent_max * \
                video_frame_gray.shape[0]:
            # Do not draw if rectangle sides too small
            if abs(max_x - min_x) > roi_percent_min * video_frame_gray.shape[1] and abs(
                    max_y - min_y) > roi_percent_min * video_frame_gray.shape[0]:
                if side_ratio_min <= (abs(max_x - min_x) / float(
                        abs(max_y - min_y))) <= side_ratio_max:  # TODO: do not draw if one side is > 2x the other sides
                    # Warp class logo perspective & flash alpha blend

                    font_scale_1 = 0.1
                    font_scale_2 = 0.2

                    cv2.putText(class_logo, text_warp1, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, font_scale_1, (255, 0, 0),
                                2,
                                cv2.LINE_AA)  # Put text
                    cv2.putText(class_logo, text_warp2, (20, 120), cv2.FONT_HERSHEY_COMPLEX_SMALL, font_scale_2,
                                (255, 0, 0), 2,
                                cv2.LINE_AA)

                    empty_class_logo = np.zeros((class_logo.shape[0], class_logo.shape[1], 3),
                                                np.uint8)  # Create white shape logo
                    empty_class_logo[:] = (255, 255, 255)

                    empty_class_logo_warped = cv2.warpPerspective(empty_class_logo, homography,
                                                                  (video_frame.shape[1], video_frame.shape[0]))  # Warp
                    class_logo_warped = cv2.warpPerspective(class_logo, homography,
                                                            (video_frame.shape[1], video_frame.shape[0]))
                    empty_class_logo_gray = cv2.cvtColor(empty_class_logo_warped, cv2.COLOR_BGR2GRAY)

                    ret, mask = cv2.threshold(empty_class_logo_gray, 10, 255, cv2.THRESH_BINARY)  # Create masks
                    mask_inv = cv2.bitwise_not(mask)
                    video_frame_background = cv2.bitwise_and(video_frame, video_frame, mask=mask_inv)
                    video_frame_foreground = cv2.bitwise_and(video_frame, video_frame, mask=mask)

                    class_logo_warped = cv2.addWeighted(class_logo_warped, 1 - flash_logo_weight_ratio,
                                                        video_frame_foreground, flash_logo_weight_ratio,
                                                        0)  # Blend & flash
                    video_frame = cv2.add(class_logo_warped, video_frame_background)

        # Put text on top left corner
        cv2.putText(video_frame, text1, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(video_frame, text2, (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 2, cv2.LINE_AA)
        text4 = "Feature matches (ORB): {}".format(len(descriptor_matches))
        cv2.putText(video_frame, text4, (50, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        text3 = "Hamming distance (sum): {}".format(int(min_total_descriptor_distance))
        cv2.putText(video_frame, text3, (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        text4 = "Class: {}".format(detected_model)
        cv2.putText(video_frame, text4, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)

        # TODO: plot graph of last 5 values?
        return video_frame


if __name__ == '__main__':

    write_video = False

    model_reference = cv2.imread("picsQr/model0.png", cv2.IMREAD_COLOR)

    capture_device = cv2.VideoCapture(0)  # TODO: Enable if want to use a local camera
    # captureDevice = cv2.VideoCapture('picsQr/vids/good8.mp4')  # TODO: Enable if u want to use a video file

    # TODO: set resolution
    capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, 1920.0)
    capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080.0)
    # capture_device.set(cv2.CAP_PROP_FPS, 15)
    # capture_device.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the autofocus off

    # lastDocumentDbData = getLastDbDocument()  # Get last document db data
    lastDocumentDbData = {'setPointMin': 0, 'setPointMax': 1, 'maxMoisture': 80,
                          'totalWater': 3000, 'waterings': 3, 'minMoisture': 30,
                          'proportional': 1.2, 'integral': 0.8, 'derivative': 1.0}  # TODO: dummy data

    if write_video:
        # Define the codec and create VideoWriter object
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter('pixelsQrToArmXYZ_AR.avi', fourcc, 20, (1920, 1080))

    flashFrame = 0

    while True:
        _, videoFrame = capture_device.read()  # Capture frame-by-frame

        # print("videoFrame.shape", videoFrame.shape)
        # break

        aligned_frame = align_images(videoFrame, model_reference, lastDocumentDbData, flashFrame)
        flashFrame += 1
        if flashFrame >= FLASH_EVERY_FRAMES * 0.75:
            flashFrame = 0
        cv2.imshow('Video Frame', aligned_frame)  # Display the resulting frame

        if write_video:
            out.write(aligned_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    capture_device.release()  # When everything done, release the capture
    if write_video:
        out.release()
    cv2.destroyAllWindows()
