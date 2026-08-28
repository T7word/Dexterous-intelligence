import cv2
import numpy as np
import glob
from math import *
import pandas as pd
import os

"""
参数
"""
num = 12  # 用于标定的图片数

# 相机内参
fx = 330.89989872
fy = 327.52309666
cx = 320.08538659
cy = 257.4981118
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]], dtype=np.float64)

# 棋盘格参数
chess_board_x_num = 10
chess_board_y_num = 7
chess_board_len = 15  # 单位棋盘格长度,mm,这里要修改
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)  # 用于查找棋盘格角点

"""
根据欧拉角计算旋转矩阵
"""


# 用于根据欧拉角计算旋转矩阵
def myRPY2R_robot(x, y, z):
    Rx = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    Ry = np.array([[cos(y), 0, sin(y)], [0, 1, 0], [-sin(y), 0, cos(y)]])
    Rz = np.array([[cos(z), -sin(z), 0], [sin(z), cos(z), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx  # @表示矩阵乘法
    return R


"""
根据平移和旋转计算齐次矩阵
"""


# 用于根据位姿计算变换矩阵
def pose_robot(x, y, z, Tx, Ty, Tz):  # 旋转角度：x,y,z;平移分量：Tx,Ty,Tz
    # thetaX = x / 180 * pi
    # thetaY = y / 180 * pi
    # thetaZ = z / 180 * pi
    thetaX = x
    thetaY = y
    thetaZ = z
    R = myRPY2R_robot(thetaX, thetaY, thetaZ)
    t = np.array([[Tx], [Ty], [Tz]])
    RT1 = np.column_stack([R, t])  # 列合并
    RT1 = np.row_stack((RT1, np.array([0, 0, 0, 1])))
    return RT1


"""
根据棋盘格图像获取标定板相对于相机的位姿
"""


# 用来从棋盘格图片得到相机外参
def get_RT_from_chessboard(img_path, chess_board_x_num, chess_board_y_num, K, chess_board_len):
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # ret, corners = cv2.findChessboardCornersSB(gray, (chess_board_y_num, chess_board_x_num), None)
    ret, corners = cv2.findChessboardCornersSB(gray, (chess_board_x_num, chess_board_y_num), None)  # 调换以后方向才对
    if ret:
        # 精细查找角点
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # 显示角点
        # cv2.drawChessboardCorners(img,(chess_board_y_num, chess_board_x_num), corners2, ret)
        cv2.drawChessboardCorners(img, (chess_board_x_num, chess_board_y_num), corners2, ret)  # 调换以后方向才对
    cv2.imshow("img", img)
    cv2.waitKey(600)

    corner_points = np.zeros((2, corners.shape[0]), dtype=np.float64)
    for i in range(corners.shape[0]):
        corner_points[:, i] = corners[i, 0, :]

    object_points = np.zeros((3, chess_board_x_num * chess_board_y_num), dtype=np.float64)
    flag = 0
    for i in range(chess_board_y_num):
        for j in range(chess_board_x_num):
            object_points[:2, flag] = np.array([(11 - j - 1) * chess_board_len, (8 - i - 1) * chess_board_len])
            flag += 1

    retval, rvec, tvec = cv2.solvePnP(object_points.T, corner_points.T, K, distCoeffs=None)

    RT = np.column_stack(((cv2.Rodrigues(rvec))[0], tvec))
    RT = np.row_stack((RT, np.array([0, 0, 0, 1])))

    return RT


"""
标定板2相机
"""
# 计算board to cam 变换矩阵
R_all_chess_to_cam_1 = []
T_all_chess_to_cam_1 = []
for i in range(num):
    image_path = './img6/{}.jpg'.format(i + 1)  # 将这个换成自己的图片路径
    RT = get_RT_from_chessboard(image_path, chess_board_x_num, chess_board_y_num, K, chess_board_len)

    R_all_chess_to_cam_1.append(RT[:3, :3])
    T_all_chess_to_cam_1.append(RT[:3, 3].reshape((3, 1)))

"""
末端2基座
"""
# 位姿参数，需根据自己的机械臂修改，pose单位mm,    ang单位rad,    每一张标定照片对应一个位姿
pose = np.array([[-133.667584, 74.2694944, 464.236604
                  ],
                 [-157.6273227, 76.17736518, 466.8356224
                  ],
                 [-131.9402425, 72.63191941, 492.3306943
                  ],
                 [-136.9445507, 62.0668343, 489.6286989
                  ],
                 [-145.8655796, 37.03782299, 485.0873202
                  ],
                 [-149.853094, 59.20460045, 528.4147787
                  ],
                 [-133.9190826, 81.51491363, 523.2420955
                  ],
                 [-189.375927, 75.58993813, 521.5422056
                  ],
                 [-96.01222951, 44.97664942, 505.2393874
                  ],
                 [-96.01222951, 44.97664942, 505.2393874
                  ],
                 [-62.49763672, 75.35100858, 513.4260665
                  ], [-48.8079461, 82.57532644, 496.1957681
                      ]])
ang = np.array([[2.474808915, 0.62242182, - 1.623352991
                 ],
                [2.447138592, 0.652031664, - 1.616136736
                 ],
                [2.273201737, 0.63836, - 1.709503428
                 ],
                [2.279888093, 0.604763772, - 1.637960099
                 ],
                [2.280804537, 0.530346341, - 1.484081679
                 ],
                [1.964845814, 0.654507664, - 1.762200505
                 ],
                [2.02939675, 0.703410756, - 1.879947594
                 ],
                [2.01799434, 0.734840235, - 1.804132473
                 ],
                [2.136827569, 0.60346747, - 1.60577055
                 ],
                [2.136827569, 0.60346747, - 1.60577055
                 ],
                [2.060414179, 0.654467672, - 1.868430843
                 ], [2.211677176, 0.637570166, - 1.871146776
                     ]])
R_all_end_to_base_1 = []
T_all_end_to_base_1 = []
# 计算end to base变换矩阵
for i in range(num):
    RT_ = pose_robot(ang[i, 0], ang[i, 1], ang[i, 2], pose[i, 0], pose[i, 1], pose[i, 2])
    RT = np.linalg.inv(RT_)  # 这里加一步求逆，求的其实是base2end
    R_all_end_to_base_1.append(RT[:3, :3])
    # T_all_end_to_base_1.append(RT[:3, 3].reshape((3, 1)))
    T_all_end_to_base_1.append(RT[:3, 3])

"""
手眼标定
"""
R, T = cv2.calibrateHandEye(R_all_end_to_base_1, T_all_end_to_base_1, R_all_chess_to_cam_1,
                            T_all_chess_to_cam_1)  # 手眼标定
RT = np.column_stack((R, T))
RT = np.row_stack((RT, np.array([0, 0, 0, 1])))  # 即为cam to end变换矩阵
print('相机相对于机械臂基座的变换矩阵为：')
print(RT)

