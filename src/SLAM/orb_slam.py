import cv2 as cv
import numpy as np

def init_orb():
    orb_detector = cv.ORB.create(nfeatures=50)
    return orb_detector

def detect_orb(img, orb_detector):
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    kp = orb_detector.detect(gray_img, None)
    kp, des = orb_detector.compute(gray_img, kp)

    return kp, des

def init_flann():
    flann_matcher = cv.FlannBasedMatcher.create()
    return flann_matcher

def match_des(flann_matcher, prev_des, current_des):
    matches = flann_matcher.knnMatch(prev_des, current_des)
    return matches

def get_points_from_matches(prev_kp, current_kp, matches):
    prev_points = [prev_kp[m.queryIdx].pt for m in matches]
    current_points = [current_kp[m.trainIdx].pt for m in matches]
    return prev_points, current_points

def calculate_3D_points(prev_points, current_points):
    projMatr = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0]])
    points_4D = cv.triangulatePoints(projMatr, projMatr, prev_points, current_points)
    points_3D = cv.convertPointsFromHomogeneous(points_4D)
    return points_3D

def calculate_transformation_matrix(points_3D, current_points, camera_matrix=np.eye(3), dist_coeffs=np.zeros(5)):
    succes, rvec, tvec = cv.solvePnP(points_3D, current_points, camera_matrix, dist_coeffs)

    if not succes:
        return None
    
    rotation_matrix, _ = cv.Rodrigues(rvec)
    transformation_matrix = np.stack((rotation_matrix, tvec), 2)

    print(transformation_matrix)

    return transformation_matrix

def calculate_camera_position(prev_position, transformation_matrix):
    prev_position_hom = cv.convertPointsToHomogeneous(prev_position)
    current_position = prev_position_hom @ transformation_matrix.T
    return current_position

def add_new_to_global_data(flann_matcher, points_3D, current_des, global_points_3D, global_des, global_weights):
    matches = flann_matcher.knnMatch(points_3D, global_points_3D)
    matches = [m for m in matches if m.distance < 20] # 20 is a threshold value to filter out really bad matches
    
    points_3D_matched = [points_3D[m.queryIdx].pt for m in matches]
    global_points_3D_matched = [global_points_3D[m.trainIdx].pt for m in matches]

    for i in range(len(points_3D)):
        point = points_3D[i]
        if point not in points_3D_matched: # Als het punt niet gematched is -> staat nog niet in de global list
            global_points_3D.append(point)
            global_des.append(current_des[i])
            global_weights.append(10)            
        else: # Anders staat die wel al in de global list -> weight verhogen
            global_index = global_points_3D.index(global_points_3D_matched[points_3D_matched.index(point)])
            global_weights[global_index] += 1

    for i in range(global_points_3D):
        point = global_points_3D[i]
        if point not in global_points_3D_matched and global_weights[i] < 20: # Punten met een weight groter dan 20 laten we (permanent) staan
            global_weights[i] -= 1

        if global_weights[i] <= 0:
            global_points_3D.pop(i)
            global_des.pop(i)
            global_weights.pop(i)

def visualize(img, kp):
    img = cv.drawKeypoints(img, kp, None)
    cv.imshow("camera", img)
    cv.waitKey(1)

def main():
    orb_detector = init_orb()
    flann_matcher = init_flann()

    global_points_3D = []
    global_des = []
    global_weights = []

    prev_kp, prev_des = None
    camera_position = [0, 0, 0]

    cap = cv.VideoCapture(0)
    while (True):
        _, img = cap.read()

        img_gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        current_kp, current_des = detect_orb(img_gray, orb_detector)
        matches = match_des(flann_matcher, prev_des, current_des)

        prev_points, current_points = get_points_from_matches(prev_kp, current_kp, matches)
        points_3D = calculate_3D_points(prev_points, current_points)

        transformation_matrix = calculate_transformation_matrix(points_3D, current_points)
        camera_position = calculate_camera_position(camera_position, transformation_matrix)

        add_new_to_global_data(flann_matcher, points_3D, current_des, global_points_3D, global_des, global_weights)

        visualize(img, current_kp)

if __name__ == "__main__":
    main()
