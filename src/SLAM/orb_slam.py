import cv2 as cv
import numpy as np

class OrbSlam():
    def __init__(self, camera_matrix = np.eye(3), dist_coeffs = np.zeros(5)):
        self.orb_detector = _init_orb()
        self.flann_matcher = _init_flann()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.prev_kp, self.prev_des = None

    def run(self, image):
        current_kp, current_des = _detect_orb(image, self.orb_detector)
        transformation_matrix = np.eye(4)
        points_relative_3d = []

        # When first time loop is running -> set prev kp, des and skip to next iteration
        if (self.prev_kp is not None or self.prev_des is not None):
            matches = _match_descriptors(self.flann_matcher, self.prev_des, current_des)
            prev_points, current_points = _get_points_from_matches(self.prev_kp, current_kp, matches)

            transformation_matrix, _ = _calculate_transformation_matrix(prev_points, current_points, self.camera_matrix, self.dist_coeffs)
            points_relative_3d = _calculate_3D_points(self.prev_kp, current_kp, transformation_matrix, self.camera_matrix)

        # Set prev to current for next loop
        self.prev_kp = current_kp
        self.prev_des = current_des

        return transformation_matrix, points_relative_3d

def _init_orb():
    orb_detector = cv.ORB.create(nfeatures=50)
    return orb_detector

def _detect_orb(img, orb_detector):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    kp = orb_detector.detect(gray_img, None)
    kp, des = orb_detector.compute(gray_img, kp)

    return kp, des

def _init_flann():
    flann_matcher = cv.FlannBasedMatcher.create()
    return flann_matcher

def _match_descriptors(flann_matcher, prev_des, current_des, threshold: float = 30):
    matches = flann_matcher.knnMatch(prev_des, current_des)
    matches = [m for m in matches if m.distance < threshold] # 30 is a threshold value to filter out really bad matches
    return matches

def _get_points_from_matches(prev_kp, current_kp, matches):
    prev_points = [prev_kp[m.queryIdx].pt for m in matches]
    current_points = [current_kp[m.trainIdx].pt for m in matches]
    return prev_points, current_points

def _calculate_3D_points(prev_points, current_points, transformation_matrix, camera_matrix=np.eye(3)):
    projMatr1 = np.hstack((camera_matrix, np.zeros((3, 1))))  # Identiteitsmatrix voor vorige frame
    projMatr2 = np.hstack((camera_matrix @ transformation_matrix[:3, :3], camera_matrix @ transformation_matrix[:3, 3].reshape(-1, 1)))

    points_4D = cv.triangulatePoints(projMatr1, projMatr2, prev_points, current_points)
    points_3D = cv.convertPointsFromHomogeneous(points_4D)

    return points_3D

def _calculate_transformation_matrix(prev_points, current_points, camera_matrix=np.eye(3), dist_coeffs=np.zeros(5)):
    E, _ = cv.findEssentialMat(prev_points, current_points, camera_matrix)
    _, R, t, _ = cv.recoverPose(E, prev_points, current_points, camera_matrix)

    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = t.flatten()

    return transformation_matrix, R, t   
