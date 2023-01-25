#!/usr/bin/env python

'''
Welcome to the SCARA packages sorting algo!
Place boxes and start program
'''

from __future__ import print_function
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from math import *
import serial
import time
from datetime import datetime

aruco_dictionary_name = "DICT_6X6_50"

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Side length of the ArUco marker in meters
aruco_marker_side_length = 0.029

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'

lastx, lasty, lastz = 0,0,0

def command(ser, command):
    start_time = datetime.now()
    ser.write(str.encode(command))
    time.sleep(1)
    while True:
        line = ser.readline()
        print(line)

        if 'ok' in str(line):
            break


def check(num):
    return num if num > 0 else pi_2 + num


pi_2 = 2 * pi


def find_angles(x, y):
    L1, L2 = 98.41, 140
    L3 = (x * x + y * y) ** 0.5
    q1_min, q1_max, q2_min, q2_max = 0, pi, 0, pi_2
    q1 = check(atan2(y, x))
    q2 = acos((L1 ** 2 - L2 ** 2 + L3 ** 2) / (2 * L1 * L3))
    Q1_1 = check(q1 - q2)
    Q1_2 = check(q1 + q2) % pi_2
    angleB = acos((L1 ** 2 + L2 ** 2 - L3 ** 2) / (2 * L1 * L2))
    Q2_1 = pi - angleB
    Q2_2 = -Q2_1

    Bx, By = L1 * cos(Q1_1), L1 * sin(Q1_1)
    Cx, Cy = Bx + L2 * cos(Q1_1 + Q2_1), By + L2 * sin(Q1_1 + Q2_1)

    Q3_1 = pi_2 - angleB
    Q3_2 = angleB

    if Bx * Cy - By * Cx < 0:
        Q3_1 = angleB
        Q3_2 = pi_2 - angleB
    Q1_1, Q1_2, Q2_1, Q2_2, Q3_1, Q3_2 = map(lambda x: round(x, 3), [Q1_1, Q1_2, Q2_1, Q2_2, Q3_1, Q3_2])
    res = [-1]
    if q1_max >= Q1_1 >= q1_min and q2_max >= Q3_1 >= q2_min:
        res = degrees(Q1_1), degrees(Q3_1)
    elif q1_max >= Q1_2 >= q1_min and q2_max >= Q3_2 >= q2_min:
        res = degrees(Q1_2), degrees(Q3_2)
    return res


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def moveangles(a, b, z, speed=3000):
    global lastx, lasty, lastz
    command(ser, "G01 X" + str(a + 20) + " Y" + str(b + a - 180 + 20) + " Z" + str(z) + " F" + str(speed) + "\r\n")

    time.sleep(max(abs(a-lastx),abs(b-lasty),abs(z-lastz))/(speed/60))
    lastx, lasty, lastz = a, b, z
def movexy(x,y,z,sevoa,speed=3000):
    x0, y0 = 10, 80
    angles = find_angles(x + x0, y + y0)
    angle = sevoa
    if angle < 0:
            angle = 180 - abs(angle)
    turn = angle + 75 + angles[1] + angles[0] - 180 +20
    if turn > 180:
        turn -= 180
    command(ser, "M280 P0 S" + str(turn) + "\r\n")
    moveangles(angles[0], angles[1], z, speed)
    time.sleep(2)

def main():
    ser = serial.Serial('/dev/tty.usbserial-AQ00C80R', 250000)
    input("Press Enter to continue...")
    time.sleep(2)
    command(ser, "G28\r\n")
    command(ser, "G01 X" + str(0) + " Y" + str(0) + " Z" + str(80) + " F5000\r\n")
    command(ser, "M280 P0 S" + str(85) + "\r\n")
    lastx, lasty, lastz = 0, 0, 0
    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
        camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()
    starta, startb = -20, -20
    print("[INFO] detecting '{}' markers...".format(
        aruco_dictionary_name))
    this_aruco_dictionary = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dictionary_name])
    this_aruco_parameters = cv2.aruco.DetectorParameters_create()

    # Start the video stream
    cap = cv2.VideoCapture(0)

    while (True):

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()

        # Detect ArUco markers in the video frame
        (corners, marker_ids, rejected) = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters,
            cameraMatrix=mtx, distCoeff=dst)

        # Check that at least one ArUco marker was detected
        if marker_ids is not None:

            # Draw a square around detected markers in the video frame
            cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)

            # Get the rotation and translation vectors
            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                aruco_marker_side_length,
                mtx,
                dst)

            arrayOftargets = []
            for i, marker_id in enumerate(marker_ids):
                # Store the translation (i.e. position) information
                transform_translation_x = tvecs[i][0][0]
                transform_translation_y = tvecs[i][0][1]
                transform_translation_z = tvecs[i][0][2]

                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()

                # Quaternion format
                transform_rotation_x = quat[0]
                transform_rotation_y = quat[1]
                transform_rotation_z = quat[2]
                transform_rotation_w = quat[3]

                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x,
                                                               transform_rotation_y,
                                                               transform_rotation_z,
                                                               transform_rotation_w)

                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                if marker_id != 0:
                    arrayOftargets.append(
                        [marker_id[0], transform_translation_x, transform_translation_y, transform_translation_z,
                         yaw_z])
                # print("transform_translation_x: {}".format(transform_translation_x))
                # print("transform_translation_y: {}".format(transform_translation_y))
                # print("transform_translation_z: {}".format(transform_translation_z))
                # print("roll_x: {}".format(roll_x))
                # print("pitch_y: {}".format(pitch_y))
                # print("yaw_z: {}".format(yaw_z))
                # print()

                # Draw the axes on the marker
                cv2.aruco.drawAxis(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
        print(arrayOftargets)
        # Display the resulting frame
        cv2.imshow('frame', frame)
        if arrayOftargets:
            print(arrayOftargets)
            for a in arrayOftargets:
                print(a)
                x, y, z, ang = a[1] * 1000, -a[2] * 1000, a[3], a[4]

                movexy(x,y,50,ang,3000)
                movexy(x, y, 0, ang, 3000)
                time.sleep(1)
                command(ser, "M280 P1 S" + str(135) + "\r\n")
                time.sleep(1)
                movexy(x, y, 100, ang, 3000)
                movexy(-180, 40, 100, ang, 3000)
                movexy(-180, 40, 50, ang, 3000)
                command(ser, "M280 P1 S" + str(60) + "\r\n")
                movexy(-180, 40, 100, ang, 3000)
                time.sleep(2)

                print('Ok')
                time.sleep(1)
            command(ser, "G01 X" + str(0) + " Y" + str(0) + " Z" + str(100) + " F" + str(3000) + "\r\n")
            input("waiting for U sir")
            #exit()
        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    print(__doc__)
    main()
