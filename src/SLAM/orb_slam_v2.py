import cv2 as cv
import numpy as np

from skimage.measure import ransac
from skimage.transform import FundamentalMatrixTransform

orb_detector = cv.ORB.create()

class OrbSlam():
    def __init__(self, camera_matrix):
        # self.orb_detector = cv.ORB.create()
        # self.flann_matcher = cv.FlannBasedMatcher.create()
        self.flann_matcher = cv.BFMatcher.create()
        self.camera_matrix = camera_matrix
        self.frames = []
        self.robot_odometry = np.eye(4)

    def run(self, image, robot_odometry):
        self.robot_odometry = self.swap_yz_axis_mat(robot_odometry)

        image = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
        frame = Frame(self.frames, image, self.camera_matrix)

        # print("frame.id =", frame.id)

        if frame.id == 0:
            return np.eye(4), np.array([])
        
        frame1 = self.frames[-1]
        frame2 = self.frames[-2]

        x1, x2, transformation_matrix = self.generate_match(frame1, frame2)
        frame1.pose = np.dot(transformation_matrix, frame2.pose)

        # print(frame1.key_points[x1].T)
        if len(x1) == 0:
            return np.eye(4), np.array([])

        points_4d = cv.triangulatePoints(frame1.pose[:3], frame2.pose[:3], frame1.key_points[x1].T, frame2.key_points[x2].T).T
        points_4d /= points_4d[:, 3:] # convert points back to 3D

        # TODO convert 3D points from opencv to sapien space by swapping y and z
        points_3d = points_4d[:, :3]
        # points_3d[:, [0, 1, 2]] = points_3d[:, [0, 2, 1]]

        return transformation_matrix, points_3d
    
    def generate_match(self, frame1, frame2):
        matches = self.flann_matcher.knnMatch(frame1.descriptors, frame2.descriptors, k=2)
        ret, x1, x2 = [], [], []

        for m, n in matches:
            if m.distance < 0.75*n.distance:
                points1 = frame1.key_points[m.queryIdx]
                points2 = frame2.key_points[m.trainIdx]

                if np.linalg.norm((points1 - points2)) < 0.1*np.linalg.norm([frame1.w, frame1.h]) and m.distance < 32:
                    if m.queryIdx not in x1 and m.trainIdx not in x2:
                        x1.append(m.queryIdx)
                        x2.append(m.trainIdx)

                        ret.append((points1, points2))
        

        # assert(len(set(x1))) == len(x1)
        # assert(len(set(x2))) == len(x2)
        # assert(len(ret)) >= 8
        
        # if len(ret) < 8 or len(set(x1)) != len(x1) or len(set(x2)) != len(x2):
        #     return [], [], np.eye(4)
        
        ret = np.array(ret)
        x1 = np.array(x1)
        x2 = np.array(x2)

        if len(ret) < 8:
            return [], [], np.eye(4)

        H, mask = cv.findHomography(ret[:, 0], ret[:, 1])
        retval, R, t, normals = cv.decomposeHomographyMat(H, self.camera_matrix)

        solution = np.eye(4)
        solution_distance = 0

        for i in range(retval):
            transformation_matrix = np.eye(4)
            transformation_matrix[0:3, 0:3] = R[i]
            transformation_matrix[0:3, 3] = t[i].flatten()

            difference = transformation_matrix - self.robot_odometry
            distance = np.linalg.norm(difference)

            if i == 0:
                solution = transformation_matrix
                solution_distance = distance
            elif distance < solution_distance:
                solution = transformation_matrix
                solution_distance = distance

        return x1[mask.ravel() == 1], x2[mask.ravel() == 1], solution

        # MANUAL VERSION -> doesn't work
        # model, f_pts = ransac((ret[:, 0], ret[:, 1]),
        #                   FundamentalMatrixTransform,
        #                   min_samples=8,
        #                   residual_threshold=0.001,
        #                   max_trials=100)
        # transformation_matrix = self.extractRt(model.params)

        # return x1[f_pts], x2[f_pts], transformation_matrix

        # FUNDAMENTAL MATRIX OPENCV VERSION -> doesn't work
        # F, mask = cv.findFundamentalMat(ret[:, 0], ret[:, 1], cv.FM_RANSAC, ransacReprojThreshold=0.001, confidence=0.99)
        # transformation_matrix = self.extractRt(F)

        # ESSENTIAL MATRIX VERSION -> doesn't work
        # E, mask = cv.findEssentialMat(ret[:, 0], ret[:, 1], self.camera_matrix)
        # _, R, t, _ = cv.recoverPose(E, ret[:, 0], ret[:, 1], self.camera_matrix)

        # transformation_matrix = np.eye(4)
        # transformation_matrix[0:3, 0:3] = R
        # transformation_matrix[0:3, 3] = t.flatten()

        # return x1[mask.ravel() == 1], x2[mask.ravel() == 1], transformation_matrix

    # Don't know if I need this yet
    def extractRt(self, F):
        W = np.asmatrix([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
        U,d,Vt = np.linalg.svd(F)
        #assert np.linalg.det(U) > 0
        if np.linalg.det(Vt) < 0:
            Vt *= -1.0
        R = np.dot(np.dot(U, W), Vt)
        if np.sum(R.diagonal()) < 0:
            R = np.dot(np.dot(U, W.T), Vt)
        t = U[:, 2]
        ret = np.eye(4)
        ret[:3, :3] = R
        ret[:3, 3] = t

        return ret
    
    def swap_yz_axis_mat(self, transformation_matrix):
        swap_matrix = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, 1, 0]
        ])

        transformation_matrix[:3, :3] = np.dot(swap_matrix, transformation_matrix[:3, :3])
        transformation_matrix[:3, 3] = np.dot(swap_matrix, transformation_matrix[:3, 3])

        return transformation_matrix
    
    def swap_yz_axis_points_3d(self, points_3d):
        swap_matrix = np.array([
            [1, 0, 0],
            [0, 0, 1],
            [0, 1, 0]
        ])

        points_3d = np.dot(swap_matrix, points_3d[:])

        return points_3d

    
class Frame():
    def __init__(self, frames, image, camera_matrix):
        self.camera_matrix = camera_matrix
        self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
        self.pose = np.eye(4)
        self.h, self.w = image.shape[0:2]
        key_points, self.descriptors = self.feature_mapping(image)
        self.key_points = self.normalize(self.camera_matrix_inv, key_points)
        self.id = len(frames)
        frames.append(self)

    def feature_mapping(self, image):
        points = cv.goodFeaturesToTrack(image, 1000, 0.01, 7)
        key_points = [cv.KeyPoint(x=f[0][0], y=f[0][1], size=20) for f in points]
        key_points, descriptors = orb_detector.compute(image, key_points)
        return np.array([(kp.pt[0], kp.pt[1]) for kp in key_points]), descriptors

    def normalize(self, count_inv, pts):
        return np.dot(count_inv, np.concatenate([pts, np.ones((pts.shape[0], 1))], axis=1).T).T[:, 0:2]