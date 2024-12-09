import cv2 as cv
import numpy as np

class OrbSlam():
    def __init__(self, camera_matrix = np.eye(3), dist_coeffs = np.zeros(5)):
        self.orb_detector = _init_orb()
        self.flann_matcher = _init_flann()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.prev_kp, self.prev_des = None, None

    def run(self, image):
        current_kp, current_des = _detect_orb(image, self.orb_detector)
        transformation_matrix = np.eye(4)
        points_relative_3d = []

        # When first time loop is running -> set prev kp, des and skip to next iteration
        if (self.prev_kp is not None or self.prev_des is not None) and (current_kp is not None or current_des is not None):
            matches = _match_descriptors(self.flann_matcher, self.prev_des, current_des)

            if len(matches) >= 8:
                prev_points, current_points = _get_points_from_matches(self.prev_kp, current_kp, matches)

                transformation_matrix, _, _ = _calculate_transformation_matrix(prev_points, current_points, self.camera_matrix, self.dist_coeffs)
                if _ is not None: # ! TODO check if this check is needed 
                    points_relative_3d = _calculate_3D_points(prev_points, current_points, transformation_matrix, self.camera_matrix)

        # Set prev to current for next loop
        self.prev_kp = current_kp
        self.prev_des = current_des

        return transformation_matrix, points_relative_3d

def _init_orb():
    orb_detector = cv.ORB.create(nfeatures=50)
    # orb_detector = cv.ORB.create(
    #     nfeatures=1000,
    #     scaleFactor=1.1,
    #     nlevels=16,
    #     edgeThreshold=15,
    #     patchSize=21
    # )
    return orb_detector

def _detect_orb(img, orb_detector):
    # gray_img = cv.cvtColor(img, cv.COLOR_RGBA2BGR)
    img = (img * 255).clip(0, 255).astype("uint8")
    kp, des = orb_detector.detectAndCompute(img, None)

    if len(kp) <= 0 or len(des) <= 0:
        return None, None

    return np.array(kp), np.array(des, dtype=np.float32)

def _init_flann():
    flann_matcher = cv.FlannBasedMatcher.create()
    return flann_matcher

def _match_descriptors(flann_matcher, prev_des, current_des, threshold: float = 100):
    bf = cv.BFMatcher.create()
    matches = bf.knnMatch(prev_des, current_des, 1)
    matches = [m[0] for m in matches if m[0].distance < threshold] # 30 is a threshold value to filter out really bad matches
    return np.array(matches).flatten()

def _get_points_from_matches(prev_kp, current_kp, matches):
    prev_points = [prev_kp[m.queryIdx].pt for m in matches]
    current_points = [current_kp[m.trainIdx].pt for m in matches]
    return np.array(prev_points, dtype=np.float32), np.array(current_points, dtype=np.float32)

def _calculate_3D_points(prev_points, current_points, transformation_matrix, camera_matrix=np.eye(3)):
    projMatr1 = np.hstack((camera_matrix, np.zeros((3, 1))))  # Identiteitsmatrix voor vorige frame
    projMatr2 = np.hstack((camera_matrix @ transformation_matrix[:3, :3], camera_matrix @ transformation_matrix[:3, 3].reshape(-1, 1)))

    points_4D = cv.triangulatePoints(projMatr1, projMatr2, prev_points.T, current_points.T)

    points_3D = cv.convertPointsFromHomogeneous(points_4D.T)

    return points_3D.squeeze()

def _calculate_transformation_matrix(prev_points, current_points, camera_matrix=np.eye(3), dist_coeffs=np.zeros(5)):
    F, _ = cv.findFundamentalMat(prev_points, current_points, cv.FM_RANSAC)
    if F.shape != (3, 3): return np.eye(4), None, None

    # E, _ = cv.findEssentialMat(prev_points, current_points, camera_matrix)
    # if E.shape != (3, 3): return np.eye(4), None, None

    E = camera_matrix.T @ F @ camera_matrix

    _, R, t, _ = cv.recoverPose(E, prev_points, current_points, camera_matrix)
    # t = t/1000

    # travel_distance = np.linalg.norm(t)
    # # print(travel_distance)

    # if travel_distance < 0.002:
    #     t = np.zeros(3)
    
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = R
    transformation_matrix[:3, 3] = t.flatten()

    swap_matrix = np.array([
        [1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    transformation_matrix = swap_matrix @ transformation_matrix @ swap_matrix.T

    return transformation_matrix, R, t   
