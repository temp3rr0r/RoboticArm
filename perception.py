from __future__ import print_function
import cv2
import numpy as np
import sys
from sklearn.externals import joblib


class Perception:
    """
    Acquires percepts via computer vision. Percept is the sensed XYZ position of an object in relation to the
    arm's frame of reference, in centimeters.
    """

    def __init__(self):
        print("--- Initializing perception...")
        self.MAX_FEATURES = 900  # 900
        self.MIN_MATCHES = 30  # 15
        self.GOOD_MATCH_PERCENT = 0.3  # 0.3
        self.FLASH_EVERY_FRAMES = 40.0
        self.MIN_DESCRIPTOR_DISTANCE_SUM = 10000
        self.use_flann = True
        self.FLANN_INDEX_LSH = 6
        self.regressor_qr_to_arm_xyz = joblib.load('modelsQr/pixels_qr_RANSACRegressor_xyz.sav')
        self.write_video = True
        self.display_output_frames = True
        self.class_logo = cv2.imread("picsQr/logoTarget.png", cv2.IMREAD_COLOR)
        self.model_reference = cv2.imread("picsQr/modelTarget.png", cv2.IMREAD_COLOR)
        self.percept_frames = 15
        self.arm_xyz_offset = [0.0, 0.0, 0.0]
        self.use_local_camera = True
        self.camera_frame_width = 1920
        self.camera_frame_height = 1080
        self.auto_focus = True

        if self.use_local_camera:
            self.capture_device = cv2.VideoCapture(0)
        else:
            self.captureDevice = cv2.VideoCapture('picsQr/vids/good8.mp4')

        self.capture_device.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_frame_width)
        self.capture_device.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_frame_height)
        self.capture_device.set(cv2.CAP_PROP_FPS, self.percept_frames)  # 15
        if not self.auto_focus:
            self.capture_device.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # turn the auto-focus off

        if self.write_video:  # Define the codec and create VideoWriter object
            self.fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.out = cv2.VideoWriter('perception.avi', self.fourcc, self.percept_frames, (1920, 1080))

        print("--- Perception initialized.")

    def align_images_get_xyz(self, video_frame, model_reference_in, flash_frame):
        """

        :param video_frame: Input raw image from the live video feed.
        :param model_reference_in: Input raw image from the target QR code to match.
        :param flash_frame: Integer counter for flashing picture over the QR code effect.
        :return: Tuple of 2 elements. First: input video frame with embedded matching information.
        Second: Predicted xyz position of the QR code.
        """
        object_xyz = [-25, -25, -25]
        flash_logo_weight_ratio = flash_frame / float(self.FLASH_EVERY_FRAMES)

        video_frame_gray = cv2.cvtColor(video_frame, cv2.COLOR_BGR2GRAY)  # Convert images to gray-scale
        # video_frame_gray = cv2.medianBlur(video_frame_gray, 3)

        orb_features = cv2.ORB_create(self.MAX_FEATURES)  # Detect ORB features and compute descriptors.
        key_points1, descriptors1 = orb_features.detectAndCompute(video_frame_gray, None)

        if self.use_flann:
            search_params = {}
            flann_params = dict(algorithm=self.FLANN_INDEX_LSH)
            descriptor_matcher = cv2.FlannBasedMatcher(flann_params,
                                                       search_params)  # bug : need to pass empty dict (#1329)
        else:
            descriptor_matcher = cv2.DescriptorMatcher_create(
                cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)  # Match features

        model_references = ["picsQr/modelTarget.png"]

        min_total_descriptor_distance = sys.maxsize
        detected_model = -1

        descriptor_matches = None
        key_points2 = None
        transformed_rectangle_points2 = None

        for i in range(len(model_references)):
            current_model_reference = cv2.imread(model_references[i], cv2.IMREAD_COLOR)  # Reference Image
            model_reference_gray = cv2.cvtColor(current_model_reference, cv2.COLOR_BGR2GRAY)
            # model_reference_gray = cv2.medianBlur(model_reference_gray, 3)

            current_key_points_2, descriptors2 = orb_features.detectAndCompute(model_reference_gray, None)
            current_descriptor_matches = descriptor_matcher.match(descriptors1, descriptors2, None)
            # current_descriptor_matches = descriptor_matcher.knnMatch(descriptors1, descriptors2, 2)  # TODO: test knn

            current_descriptor_matches.sort(key=lambda x_point: x_point.distance,
                                            reverse=False)  # Sort matches by score
            num_good_matches = int(len(current_descriptor_matches) * self.GOOD_MATCH_PERCENT)  # Remove mediocre matches
            current_descriptor_matches = current_descriptor_matches[:num_good_matches]
            total_descriptor_distance = 0
            for x in current_descriptor_matches:
                total_descriptor_distance += x.distance
            if total_descriptor_distance < min_total_descriptor_distance:
                min_total_descriptor_distance = total_descriptor_distance
                key_points2 = current_key_points_2
                model_reference_in = current_model_reference
                descriptor_matches = current_descriptor_matches
                detected_model = i

        if len(descriptor_matches) < self.MIN_MATCHES or min_total_descriptor_distance \
                < self.MIN_DESCRIPTOR_DISTANCE_SUM:
            return video_frame  # Not good detection
        else:
            matched_points1 = np.zeros((len(descriptor_matches), 2),
                                       dtype=np.float32)  # Extract location of good matches
            matched_points2 = np.zeros((len(descriptor_matches), 2), dtype=np.float32)

            for i, match in enumerate(descriptor_matches):
                matched_points1[i, :] = key_points1[match.queryIdx].pt
                matched_points2[i, :] = key_points2[match.trainIdx].pt

            height, width, _ = model_reference_in.shape
            homography, _ = cv2.findHomography(matched_points2, matched_points1, cv2.RANSAC)  # Find homography
            rectangle_points = np.float32([[0, 0], [0, height - 1], [width - 1, height - 1], [width - 1, 0]]).reshape(
                -1, 1,
                2)
            transformed_rectangle_points = cv2.perspectiveTransform(rectangle_points, homography)  # Use homography

            # Draw rectangle only if within a specific ROI %
            roi_percent_min = 0.0002  # 0.1
            roi_percent_max = 0.001  # 0.25
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
            side_ratio_min = 0.6
            side_ratio_max = 1.2
            if max_x < (1 - roi_percent_max) * video_frame_gray.shape[1] and min_x > roi_percent_max * \
                    video_frame_gray.shape[1] and max_y < (1 - roi_percent_max) * video_frame_gray.shape[
                0] and min_y > roi_percent_max * video_frame_gray.shape[0]:
                # Do not draw if rectangle sides too small
                if abs(max_x - min_x) > roi_percent_min * video_frame_gray.shape[1] and abs(
                        max_y - min_y) > roi_percent_min * video_frame_gray.shape[0]:
                    if side_ratio_min <= (abs(max_x - min_x) / float(
                            abs(
                                max_y - min_y))) <= side_ratio_max:  # TODO: don't draw if one side is > 2x the others
                        transformedRectangle = cv2.polylines(video_frame, [np.int32(transformed_rectangle_points)],
                                                             True,
                                                             (0, 0, 255), 3,
                                                             cv2.LINE_AA)  # Draw rectangle of the found model in frame

            text1 = ""
            text2 = ""
            text_warp1 = ""
            text_warp2 = ""
            if detected_model == 0:
                text1 = 'Target vs arm (RANSAC): '
                text2 = ''
                last_qr_position = [[transformed_rectangle_points[0][0][0], transformed_rectangle_points[0][0][1],
                                     transformed_rectangle_points[1][0][0], transformed_rectangle_points[1][0][1],
                                     transformed_rectangle_points[2][0][0], transformed_rectangle_points[2][0][1],
                                     transformed_rectangle_points[3][0][0], transformed_rectangle_points[3][0][1]]]
                regressor_predicted_arm_xyz = self.regressor_qr_to_arm_xyz.predict(last_qr_position)  # TODO: z pos
                regressor_predicted_arm_xyz[0] += self.arm_xyz_offset
                regressor_predicted_arm_xyz = np.round(regressor_predicted_arm_xyz, 1)

                object_xyz = [regressor_predicted_arm_xyz[0][0],
                              regressor_predicted_arm_xyz[0][1],
                              regressor_predicted_arm_xyz[0][2]]

                text2 = text2 + "{} {} {} cm".format(object_xyz[0],
                                                     object_xyz[1],
                                                     object_xyz[2])
                text_warp1 = "Target"
                text_warp2 = "object"

            if max_x < (1 - roi_percent_max) * video_frame_gray.shape[1] and min_x > roi_percent_max * \
                    video_frame_gray.shape[1] \
                    and max_y < (1 - roi_percent_max) * video_frame_gray.shape[0] and min_y > roi_percent_max * \
                    video_frame_gray.shape[0]:
                # Do not draw if rectangle sides too small
                if abs(max_x - min_x) > roi_percent_min * video_frame_gray.shape[1] and abs(
                        max_y - min_y) > roi_percent_min * video_frame_gray.shape[0]:
                    if side_ratio_min <= (abs(max_x - min_x) / float(
                            abs(
                                max_y - min_y))) <= side_ratio_max:  # TODO: do not draw if one side is > 2x the other sides
                        # Warp class logo perspective & flash alpha blend

                        cv2.putText(self.class_logo, text_warp1, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0),
                                    2,
                                    cv2.LINE_AA)  # Put text
                        cv2.putText(self.class_logo, text_warp2, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                    (255, 0, 0), 2,
                                    cv2.LINE_AA)

                        empty_class_logo = np.zeros((self.class_logo.shape[0], self.class_logo.shape[1], 3),
                                                    np.uint8)  # Create white shape logo
                        empty_class_logo[:] = (255, 255, 255)

                        empty_class_logo_warped = cv2.warpPerspective(empty_class_logo, homography,
                                                                      (video_frame.shape[1],
                                                                       video_frame.shape[0]))  # Warp
                        class_logo_warped = cv2.warpPerspective(self.class_logo, homography,
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
            text3 = "FLANN matcher distance (sum): {}".format(int(min_total_descriptor_distance))
            cv2.putText(video_frame, text3, (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            text4 = "Class: {}".format(detected_model)
            cv2.putText(video_frame, text4, (50, 260), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2, cv2.LINE_AA)

            return video_frame, object_xyz

    def destroy(self):
        """
        Releases the capture device and the write video out device (if storing to a video on disk).
        :return:
        """
        self.capture_device.release()  # When everything done, release the capture
        if self.write_video:
            self.out.release()
        cv2.destroyAllWindows()

    def get_percept(self):
        """
        Returns the mean perceived position XYZ in cm, of the detected object.
        :return: List of 3 XYZ float values, centimeters of the object vs the arm frame of reference.
        """
        arm_object_xyz_list = []
        flash_frame = 0
        for _ in range(self.percept_frames):
            _, video_frame = self.capture_device.read()  # Capture frame-by-frame
            try:
                aligned_frame, arm_object_xyz = self.align_images_get_xyz(video_frame, self.model_reference,
                                                                          flash_frame)
                arm_object_xyz_list.append(arm_object_xyz)
            except ValueError as e:
                print("ValueError Exception: {}".format(str(e)))  # TODO: why too many values error?

            flash_frame += 1
            if flash_frame >= self.FLASH_EVERY_FRAMES * 0.75:
                flash_frame = 0
            if self.display_output_frames:
                cv2.imshow('Video Frame', aligned_frame)  # Display the resulting frame

            if self.write_video:
                self.out.write(aligned_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        return np.round(np.mean(arm_object_xyz_list, axis=0), 1).tolist()


if __name__ == '__main__':

    "Sequence for testing"
    perception = Perception()
    import time
    steps = 500
    for j in range(steps):
        time.sleep(0.1)
        xyz = perception.get_percept()  # TODO: in BDI: if xyz outside bounds -> world model: no object detected
        print("Percept({}, mean of {}): {} cm".format(j, perception.percept_frames, xyz))  # TODO: sliding window mean
    perception.destroy()
