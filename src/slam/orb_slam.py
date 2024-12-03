import cv2 as cv
import numpy as np

class OrbSlam():
    def __init__(self, camera_matrix = np.eye(3), dist_coeffs = np.zeros(5)):
        self.orb_detector = _init_orb()
        self.flann_matcher = _init_flann()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.prev_kp, self.prev_des= None, None
        self.depthmap = None

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

    def _generate_depthmap(self, points_3d, image_shape, camera_matrix):
        """
        Generate depthmap with 3d points
        points_3d: points coming from observation of environment
        image_shape: shape of image
        camera_matrix = 3x3 matrix of the camera to project from 3d to 2d
        
        Returns 2D array with same shape as image. 
        Contains closest z-val or np.inf
        z of depthmap can be asked by: z=depthmap[x,y]
        """
        #:2 because colored img, height&width van image
        h,w = image_shape[:2]
        #vul depthmap met inf
        depthmap = np.full((h,w), np.inf)
        
        for point in points_3d:
            x,y,z = point[0], point[1],point[2]
            #ignore negative values for z => behind camera
            if z <=0:
                continue
            #project punt naar 2d vlak met cam matrix
            pixel_coords = camera_matrix @ np.array([x,y,z])[:3]
            pixel_coords /= pixel_coords[2]
            u,v= int(pixel_coords[0]), int(pixel_coords[1])
            #check of punt binnen de grenzen ligt
            if 0 <=u < w and 0 <= v<h:
                #select closest
                depthmap[v,u] = min(depthmap[v,u],z)
        self.depthmap = depthmap

def _init_orb():
    orb_detector = cv.ORB.create(nfeatures=50)
    return orb_detector

def _detect_orb(img, orb_detector):
    #gray_img = img
#    img = cv.cvtColor(image, cv.COLOR_RGBA2GRAY)
    if img.dtype != np.uint8:
        img = (img * 255).astype(np.uint8)
    gray_img = cv.cvtColor(img, cv.COLOR_RGBA2GRAY)
    kp = orb_detector.detect(gray_img, None)
    kp, des = orb_detector.compute(gray_img, kp)
    return kp, des

def _init_flann():
    flann_matcher = cv.FlannBasedMatcher.create()
    return flann_matcher

def _match_descriptors(flann_matcher, prev_des, current_des, threshold: float = 30):
    matches = flann_matcher.knnMatch(prev_des, current_des,k=2)
    matches = [m for m in matches if m.distance < threshold] # 30 is a threshold value to filter out really bad matches
    return matches

def _get_points_from_matches(prev_kp, current_kp, matches):
    prev_points = [prev_kp[m.queryIdx].pt for m in matches]
    current_points = [current_kp[m.trainIdx].pt for m in matches]
    return np.array(prev_points), np.array(current_points)

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
