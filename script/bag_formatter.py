#!/usr/bin/env python

import math
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
# from livox_ros_driver2.msg import CustomMsg
import rosbag
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from tqdm import tqdm
import os
import numpy as np
import pandas as pd
from gps_time import GPSTime
from datetime import datetime, timedelta
from pnt_msgs.msg import GNSS


def resize_image(image_msg, scale_factor=0.5):
    """
    Resize the image to a specified scale factor (default is half the original size).
    """
    # Convert ROS image message to OpenCV image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="mono8")

    # Resize image
    new_width = int(cv_image.shape[1] * scale_factor)
    new_height = int(cv_image.shape[0] * scale_factor)
    resized_image = cv2.resize(cv_image, (new_width, new_height))

    # Convert the resized image back to a ROS Image message
    resized_image_msg = bridge.cv2_to_imgmsg(
        resized_image, encoding="mono8", header=image_msg.header)
    return resized_image_msg


def process_rosbag_img(input_bag_file, output_bag_file):
    """
    Processes the input rosbag, resizes all image messages, and saves them to a new rosbag.
    """
    # Open the input and output rosbag files
    input_bag = rosbag.Bag(input_bag_file, 'r')
    output_bag = rosbag.Bag(output_bag_file, 'w')

    # Read and process each message in the bag
    for topic, msg, t in tqdm(input_bag.read_messages()):
        if topic == "/sensor/cam0" or topic == "/sensor/cam1":  # Check if the message is an image
            # print(f"Processing image on topic: {topic}")
            resized_image_msg = resize_image(msg)
            # Write the resized image to the output bag
            output_bag.write(topic, resized_image_msg, t)
        else:
            # For non-image messages, copy them as-is
            output_bag.write(topic, msg, t)

    # Close the bags
    input_bag.close()
    output_bag.close()
    print(f"Processing complete! Output saved to {output_bag_file}")


# def convert_custom_to_pointcloud2(msg):
#     header = Header()
#     header.stamp = msg.header.stamp
#     header.frame_id = msg.header.frame_id

#     points = []
#     for pt in msg.points:
#         # (x, y, z, intensity)
#         points.append([pt.x, pt.y, pt.z, pt.reflectivity])

#     fields = [
#         PointField('x', 0,  PointField.FLOAT32, 1),
#         PointField('y', 4,  PointField.FLOAT32, 1),
#         PointField('z', 8,  PointField.FLOAT32, 1),
#         PointField('intensity', 12, PointField.FLOAT32, 1)
#     ]

#     pointcloud2_msg = pc2.create_cloud(header, fields, points)
#     return pointcloud2_msg


def custommsg_to_pointcloud2(msg):
    header = Header()
    header.stamp = msg.header.stamp
    header.frame_id = msg.header.frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('time', 16, PointField.FLOAT32, 1),
        PointField('ring', 20, PointField.UINT16, 1),
    ]

    points = []
    for pt in msg.points:
        x = pt.x
        y = pt.y
        z = pt.z
        intensity = float(pt.reflectivity)
        time = pt.offset_time * 1e-6  # us to s
        ring = pt.line
        points.append([x, y, z, intensity, time, ring])

    cloud = pc2.create_cloud(header, fields, points)
    return cloud


def custommsg_to_pointcloud2_delnan(msg):
    header = Header()
    header.stamp = msg.header.stamp
    header.frame_id = msg.header.frame_id

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('time', 16, PointField.FLOAT32, 1),
        PointField('ring', 20, PointField.UINT16, 1),
    ]

    points = []
    for pt in msg.points:
        x = pt.x
        y = pt.y
        z = pt.z

        # 检查是否包含 NaN 或 Inf
        if any([math.isnan(v) or math.isinf(v) for v in [x, y, z]]):
            continue  # 跳过无效点

        intensity = float(pt.reflectivity)
        time = pt.offset_time * 1e-6  # us to s
        ring = pt.line
        points.append([x, y, z, intensity, time, ring])

    if not points:
        print("Warning: No valid points in CustomMsg after filtering NaN/Inf!")

    cloud = pc2.create_cloud(header, fields, points)
    return cloud


def process_rosbag_lidar(input_bag_path, output_bag_path, input_topic="/livox/lidar", output_topic="/livox/pointcloud2"):
    in_bag = rosbag.Bag(input_bag_path, 'r')
    out_bag = rosbag.Bag(output_bag_path, 'w')

    for topic, msg, t in tqdm(in_bag.read_messages()):
        if topic == input_topic:
            pc2_msg = custommsg_to_pointcloud2_delnan(msg)
            out_bag.write(output_topic, pc2_msg, t)
        elif topic == "/livox/imu":
            out_bag.write(topic, msg, t)

    in_bag.close()
    out_bag.close()
    print("✅ 转换完成，保存到：", output_bag_path)


def main_merge_rtk(front_bag_path, backend_bag_path):
    rtk_msgs = []
    in_bag = rosbag.Bag(backend_bag_path, 'r')
    for topic, msg, t in tqdm(in_bag.read_messages()):
        if topic == "/pose/gnss":
            rtk_msgs.append(msg)
    in_bag.close()

    rtk_bag_path = os.path.join(
        os.path.dirname(front_bag_path), "rtk-"+os.path.basename(front_bag_path))
    i = 0
    out_bag = rosbag.Bag(rtk_bag_path, 'w')
    in_bag = rosbag.Bag(front_bag_path, 'r')

    t_imu = -1

    for topic, msg, t in tqdm(in_bag.read_messages()):
        if topic == "/byh_uav/ZEDF9P" and i < len(rtk_msgs):
            out_bag.write(topic, msg, t)
            while i < len(rtk_msgs) and (rtk_msgs[i].timestamp < msg.pps_gps_time - 0.1):
                i += 1
            if i >= len(rtk_msgs):
                continue
            if abs(rtk_msgs[i].timestamp - msg.pps_gps_time) >= 0.1:
                print("unmatched timestamp: ",
                      rtk_msgs[i].timestamp, msg.pps_gps_time)
            else:
                out_bag.write("/pose/gnss", rtk_msgs[i], t)
        elif topic in ["/byh_uav/ADIS16470", "/sensor/cam0", "/sensor/cam1"]:
            if (topic == "/sensor/cam0" or topic == "/sensor/cam1") \
                    and (msg.header.stamp.to_sec() == 291299.022783311 or msg.header.stamp.to_sec() == 291299.072784313):
                msg.header.stamp.secs = msg.header.stamp.secs - 1
                print("===> change cam timestamp to ",
                      msg.header.stamp.to_sec())
            if topic == "/byh_uav/ADIS16470":
                if t_imu > 0 and msg.accel_gps_time > t_imu + 1:
                    msg.accel_gps_time = msg.accel_gps_time - 1.0
                    print("===> change IMU timestamp to ",
                          msg.accel_gps_time)
                t_imu = msg.accel_gps_time

            out_bag.write(topic, msg, t)

    in_bag.close()
    out_bag.close()

    print("✅ === Finished, saved to ", rtk_bag_path)


def main_KB2Pinhole(bag_path):

    K1 = np.array([[205.8448679402933, 0.0, 253.47590627469424],
                  [0.0, 205.70422737860193, 189.8466187556357],
                  [0.0, 0.0, 1.0]])
    D1 = np.array([0.02051433674991833, 0.001568118097478111, -
                  0.0006982566579830167, -0.00010630594438177424])  # fisheye D: k1, k2, k3, k4

    K1_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K1, D1, (512, 284), np.eye(3), balance=0.0)
    map1_cam0, map2_cam0 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 284), cv2.CV_16SC2)

    K2 = np.array([[204.89893740600002, 0.0, 250.99909265598245],
                  [0.0, 204.96142209115186, 187.575907167867],
                  [0.0, 0.0, 1.0]])
    D2 = np.array([0.02100631953324343, 0.000844410782354286,
                  0.0008482064521798298, -0.000751086948716441])  # fisheye D: k1, k2, k3, k4

    K2_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K2, D2, (512, 284), np.eye(3), balance=0.0)
    map1_cam1, map2_cam1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 284), cv2.CV_16SC2)

    bridge = CvBridge()

    output_bag_path = os.path.join(
        os.path.dirname(bag_path), "pin-"+os.path.basename(bag_path))

    out_bag = rosbag.Bag(output_bag_path, 'w')
    in_bag = rosbag.Bag(bag_path, 'r')

    for topic, msg, t in tqdm(in_bag.read_messages()):
        # 1. 获取图像
        if topic == "/sensor/cam0" or topic == "/sensor/cam1":
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            if topic == "/sensor/cam0":
                undistorted = cv2.remap(
                    cv_img, map1_cam0, map2_cam0, interpolation=cv2.INTER_LINEAR)

            if topic == "/sensor/cam1":
                undistorted = cv2.remap(
                    cv_img, map1_cam1, map2_cam1, interpolation=cv2.INTER_LINEAR)

            undistorted_image_msg = bridge.cv2_to_imgmsg(
                undistorted, encoding="mono8", header=msg.header)

            out_bag.write(topic, undistorted_image_msg, t)

        else:

            out_bag.write(topic, msg, t)

            # 5. 输出针孔模型相机参数
    print("=== PINHOLE 相机参数 ===")
    print("K1_new=", K1_new)
    print("K2_new=", K2_new)

    out_bag.close()
    in_bag.close()


def main_euroc_folder(bag_file):
    bridge = CvBridge()

    output_folder = os.path.dirname(bag_file)

    image_folder_left = os.path.join(output_folder, "00/cam0/data")

    image_folder_right = os.path.join(output_folder, "00/cam1/data")
    imu_file = os.path.join(output_folder, "00/imu0.csv")
    time_file = os.path.join(output_folder, "00/align_timestamp.txt")

    # 创建文件夹
    os.makedirs(image_folder_left, exist_ok=True)
    os.makedirs(image_folder_right, exist_ok=True)

    bag = rosbag.Bag(bag_file, 'r')

    # 用于存储 IMU 数据
    imu_data = []
    f_time = open(time_file, "w")
    align_time = -1
    cv_image_left = None
    cv_image_right = None
    # 打开 rosbag 文件，遍历所有消息
    for topic, msg, t in tqdm(bag.read_messages()):
        # if msg.header.seq < 21282:
        #     continue

        if topic == "/sensor/cam0":
            # 获取时间戳
            timestamp = msg.header.stamp.to_nsec()
            try:
                image = bridge.imgmsg_to_cv2(msg, "mono8")
            except Exception as e:
                print(f"Error converting image: {e}")
                continue
            if align_time == -1:
                align_time = timestamp
                cv_image_left = image

            elif align_time == timestamp:
                cv_image_left = image
                image_filename = os.path.join(
                    image_folder_left, f"{align_time}.png")
                cv2.imwrite(image_filename, cv_image_left)  # 保存左图像
                image_filename = os.path.join(
                    image_folder_right, f"{align_time}.png")
                cv2.imwrite(image_filename, cv_image_right)  # 保存左图像

                f_time.write(f"{align_time}\n")

                align_time = -1
                cv_image_left = None
                cv_image_right = None
            else:
                print("misalignment at {}, {}" % align_time, timestamp)
                align_time = -1
                cv_image_left = None
                cv_image_right = None

        elif topic == "/sensor/cam1":
            # 获取时间戳
            timestamp = msg.header.stamp.to_nsec()
            try:
                image = bridge.imgmsg_to_cv2(msg, "mono8")
            except Exception as e:
                print(f"Error converting image: {e}")
                continue
            if align_time == -1:
                align_time = timestamp
                cv_image_right = image

            elif align_time == timestamp:
                cv_image_right = image
                image_filename = os.path.join(
                    image_folder_left, f"{align_time}.png")
                cv2.imwrite(image_filename, cv_image_left)  # 保存左图像
                image_filename = os.path.join(
                    image_folder_right, f"{align_time}.png")
                cv2.imwrite(image_filename, cv_image_right)  # 保存左图像

                f_time.write(f"{align_time}\n")

                align_time = -1
                cv_image_left = None
                cv_image_right = None
            else:
                print("misalignment at {}, {}", align_time, timestamp)
                align_time = -1
                cv_image_left = None
                cv_image_right = None

        # 处理 IMU 数据
        elif topic == "/byh_uav/ADIS16470":
            timestamp = int(msg.accel_gps_time * 1e9)
            accel_x = msg.linear_acceleration.x
            accel_y = msg.linear_acceleration.y
            accel_z = msg.linear_acceleration.z
            gyro_x = msg.angular_velocity.x
            gyro_y = msg.angular_velocity.y
            gyro_z = msg.angular_velocity.z

            # 存储 IMU 数据
            imu_data.append([timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y,
                            accel_z])

    # 保存 IMU 数据为 CSV
    imu_df = pd.DataFrame(imu_data, columns=[
        "#timestamp", "gyro_x", "gyro_y", "gyro_z", "accel_x", "accel_y", "accel_z"])
    imu_df.to_csv(imu_file, index=False)

    f_time.close()

    print("✅ === Finished, EuRoC dataset saved to ", output_folder)


def main_lidar_batch():
    # input_bag_file = "/mnt/f/PNT/实验/1128/frontend_2024-11-28-15-30-45.bag"
    # output_bag_file = "/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi/2024-11-28-15-30-45.bag"

    # process_rosbag_img(input_bag_file, output_bag_file)
    file_list = [
        # "/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi/2024-11-28-15-30-45.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1128_Ground_Youyi/2024-11-28-15-46-44.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1128_Ground_Youyi/2024-11-28-16-03-25.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/05_1128_Ground_Youyi/2024-11-28-16-36-01.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/01_1208_Ground_Yunzhi/2024-12-08-19-42-27.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1208_Ground_Yunzhi/2024-12-08-19-57-35.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1209_Ground_Yunzhi/2024-12-09-11-05-23.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/04_1209_Ground_Yunzhi/2024-12-09-11-17-14.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/05_1209_Ground_Yunzhi/2024-12-09-11-28-13.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/06_1209_Ground_Yunzhi/2024-12-09-12-23-38.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/07_1209_Ground_Yunzhi/2024-12-09-12-32-18.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/08_1209_Ground_Yunzhi/2024-12-09-12-40-00.bag",
        # "/mnt/f/PNT/实验/NMS-PNT/09_1209_Ground_Yunzhi/2024-12-09-13-02-07.bag"
        # "/mnt/f/PNT/实验/NMS-PNT/04_0521_Ground_Yunzhi/2025-05-21-10-42-54.bag",
        "/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi/2025-05-21-12-06-50.bag",

    ]
    for input_bag_path in file_list:
        main_lidar(input_bag_path)


def main_lidar(front_bag_path):
    dir_name = os.path.dirname(front_bag_path)

    output_bag_path = os.path.join(
        dir_name, "pc2-" + os.path.basename(front_bag_path))
    print(front_bag_path, "  ===>  ", output_bag_path)
    process_rosbag_lidar(front_bag_path, output_bag_path)


def main_param_fromat():
    mat = np.array([[0.99977676, -0.00225272,  0.02100853,  0.2767191],
                    [0.0022743, 0.99999691, -0.00100355, 0.00183668],
                    [-0.02100621, 0.00105111,  0.99977879, -0.00489098],
                    [0.0,    0.0,          0.0,          1.0]])
    mat_inv = np.linalg.inv(mat)

    # mat1 = np.array([[0.01423926, -0.00527072,  0.99988472,  0.28195979],
    #                  [0.99983308,  0.01152409, -0.01417778,  0.08149144],
    #                  [-0.01144803,  0.9999197,   0.00543394, -0.02714311],
    #                  [0.0,          0.0,          0.0,         1.0]])
    mat1 = np.array([-0.00745597, -0.01649116, 0.99983621, 0.28638546,
                     0.99995738, -0.00556741, 0.00736505, -0.18632201,
                     0.00544504, 0.99984851, 0.01653197, -0.02842654,
                     0.00000000, 0.00000000, 0.000000000, 1.00000000])
    mat1.reshape((4, 4))
    from scipy.spatial.transform import Rotation as R
    mr = R.from_matrix(mat1[:-1, :-1])
    mr.as_quat()

    print("=== end ===")


def main_icgvins(front_bag_path, back_bag_path):

    # load rtk msgs
    rtk_msgs = []
    back_bag = rosbag.Bag(back_bag_path, 'r')
    for topic, msg, t in tqdm(back_bag.read_messages()):
        if topic == "/pose/gnss":
            rtk_msgs.append(msg)
    back_bag.close()

    # calculate undistort paramters
    K1 = np.array([[205.5132896, 0.0, 254.24112619],
                  [0.0, 205.44721754, 189.91968545],
                  [0.0, 0.0, 1.0]])
    # fisheye D: k1, k2, k3, k4
    D1 = np.array([0.02125097, 0.00361244, -0.00390146, 0.00100103])
    K1_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K1, D1, (512, 384), np.eye(3), balance=0.0)
    map1_cam0, map2_cam0 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 384), cv2.CV_32FC1)

    K2 = np.array([[204.62394729, 0.0, 251.67155014],
                  [0.0, 204.5888984, 187.42813825],
                  [0.0, 0.0, 1.0]])
    # fisheye D: k1, k2, k3, k4
    D2 = np.array([0.02245296, 0.00040792, -0.00046961, 0.00007108])
    K2_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K2, D2, (512, 384), np.eye(3), balance=0.0)
    map1_cam1, map2_cam1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 384), cv2.CV_32FC1)
    bridge = CvBridge()

    # setup output path
    gvins_bag_path = os.path.join(
        os.path.dirname(front_bag_path), "gvins-"+os.path.basename(front_bag_path))
    i = 0
    gvins_bag = rosbag.Bag(gvins_bag_path, 'w')
    front_bag = rosbag.Bag(front_bag_path, 'r')
    t_imu = -1

    for topic, msg, t in tqdm(front_bag.read_messages()):
        if topic == "/byh_uav/ZEDF9P" and i < len(rtk_msgs):
            gvins_bag.write(topic, msg, t)
            while i < len(rtk_msgs) and (rtk_msgs[i].timestamp < msg.pps_gps_time - 0.1):
                i += 1
            if i >= len(rtk_msgs):
                continue
            if abs(rtk_msgs[i].timestamp - msg.pps_gps_time) >= 0.1:
                print("unmatched timestamp: ",
                      rtk_msgs[i].timestamp, msg.pps_gps_time)
            else:
                gvins_bag.write("/pose/gnss", rtk_msgs[i], t)

        elif topic == "/sensor/cam0" or topic == "/sensor/cam1":
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            if topic == "/sensor/cam0":
                undistorted = cv2.remap(
                    cv_img, map1_cam0, map2_cam0, interpolation=cv2.INTER_LINEAR)

            if topic == "/sensor/cam1":
                undistorted = cv2.remap(
                    cv_img, map1_cam1, map2_cam1, interpolation=cv2.INTER_LINEAR)

            undistorted_image_msg = bridge.cv2_to_imgmsg(
                undistorted, encoding="mono8", header=msg.header)
            if (msg.header.stamp.to_sec() == 291299.022783311 or msg.header.stamp.to_sec() == 291299.072784313):
                msg.header.stamp.secs = msg.header.stamp.secs - 1
                print("===> change cam timestamp to ",
                      msg.header.stamp.to_sec())
            gvins_bag.write(topic, undistorted_image_msg, t)

        elif topic == "/byh_uav/ADIS16470":
            if t_imu > 0 and msg.accel_gps_time > t_imu + 1:
                msg.accel_gps_time = msg.accel_gps_time - 1.0
                print("===> change IMU timestamp to ",
                      msg.accel_gps_time)
            t_imu = msg.accel_gps_time
            gvins_bag.write(topic, msg, t)

    front_bag.close()
    gvins_bag.close()

    print("✅ === Finished, saved to ", gvins_bag_path)

    print("=== PINHOLE 相机参数 ===")
    print("K1_new=", K1_new)
    print("K2_new=", K2_new)


def main_icgvins_rtklib(front_bag_path, rtkpos_path):
    df_rtk = pd.read_csv(rtkpos_path, sep='\s+')
    df_rtk['datetime_str'] = df_rtk['%'] + ' ' + df_rtk['GPST']
    df_rtk['datetime'] = pd.to_datetime(
        df_rtk['datetime_str'], format='%Y/%m/%d %H:%M:%S.%f')
    df_rtk["datetime"] = df_rtk["datetime"].apply(
        lambda dt: GPSTime.from_datetime(dt).time_of_week)
    # calculate undistort paramters
    K1 = np.array([[206.3737365, 0.0, 251.0481857],
                  [0.0, 206.2609492, 188.8251849],
                  [0.0, 0.0, 1.0]])
    # fisheye D: k1, k2, k3, k4
    D1 = np.array([0.0185191, 0.0064746, -0.0049914, 0.00135775])
    K1_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K1, D1, (512, 384), np.eye(3), balance=0.0)
    map1_cam0, map2_cam0 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 384), cv2.CV_32FC1)

    K2 = np.array([[205.3186390, 0.0, 249.3016554],
                  [0.0, 205.2913999, 188.5577704],
                  [0.0, 0.0, 1.0]])
    # fisheye D: k1, k2, k3, k4
    D2 = np.array([0.0201040, 0.0058329, -0.0065461, 0.0029836])
    K2_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K2, D2, (512, 384), np.eye(3), balance=0.0)
    map1_cam1, map2_cam1 = cv2.fisheye.initUndistortRectifyMap(
        K1, D1, np.eye(3), K1_new, (512, 384), cv2.CV_32FC1)
    bridge = CvBridge()

    # setup output path
    gvins_bag_path = os.path.join(
        os.path.dirname(front_bag_path), "gvins-"+os.path.basename(front_bag_path))
    i = 0
    gvins_bag = rosbag.Bag(gvins_bag_path, 'w')
    front_bag = rosbag.Bag(front_bag_path, 'r')

    for topic, msg, t in tqdm(front_bag.read_messages()):
        if topic == "/byh_uav/ZEDF9P" and i < len(df_rtk):
            gvins_bag.write(topic, msg, t)
            while i < len(df_rtk) and (df_rtk["datetime"][i] < msg.pps_gps_time - 0.1):
                i += 1
            if i >= len(df_rtk):
                continue
            if abs(df_rtk["datetime"][i] - msg.pps_gps_time) >= 0.1:
                print("unmatched timestamp: ",
                      df_rtk["datetime"][i], msg.pps_gps_time)
            else:
                rtk_msg = GNSS()
                rtk_msg.timestamp = df_rtk["datetime"][i]
                rtk_msg.latitude = df_rtk["latitude(deg)"][i]
                rtk_msg.longitude = df_rtk["longitude(deg)"][i]
                rtk_msg.height = df_rtk["height(m)"][i]
                rtk_msg.pD_std = df_rtk["sdu(m)"][i]
                rtk_msg.pE_std = df_rtk["sde(m)"][i]
                rtk_msg.pN_std = df_rtk["sdn(m)"][i]
                rtk_msg.pED_cov = df_rtk["sdeu(m)"][i]
                rtk_msg.pNE_cov = df_rtk["sdne(m)"][i]
                rtk_msg.pND_cov = df_rtk["sdun(m)"][i]
                rtk_msg.vN = df_rtk["vn(m/s)"][i]
                rtk_msg.vE = df_rtk["ve(m/s)"][i]
                rtk_msg.vD = df_rtk["vu(m/s)"][i]
                gvins_bag.write("/pose/gnss", rtk_msg, t)

        elif topic == "/sensor/cam0" or topic == "/sensor/cam1":
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            if topic == "/sensor/cam0":
                undistorted = cv2.remap(
                    cv_img, map1_cam0, map2_cam0, interpolation=cv2.INTER_LINEAR)
                undistorted_image_msg = bridge.cv2_to_imgmsg(
                    undistorted, encoding="mono8", header=msg.header)
                gvins_bag.write(topic+"_undis", undistorted_image_msg, t)

            gvins_bag.write(topic, msg, t)

        elif topic == "/byh_uav/ADIS16470":
            gvins_bag.write(topic, msg, t)

    front_bag.close()
    gvins_bag.close()

    print("✅ === Finished, saved to ", gvins_bag_path)

    print("=== PINHOLE 相机参数 ===")
    print("K1_new=", K1_new)
    print("K2_new=", K2_new)

# convert bag to euroc folder


def main_euroc_batch():
    # bag = "/mnt/f/PNT/实验/1128/frontend_2024-11-28-16-03-25.bag"
    # folder = "/mnt/f/PNT/实验/NMS-PNT/03_1128_Ground_Youyi"
    # to_euroc_folder(bag, folder)

    folder_list = [
        # "/mnt/f/PNT/实验/NMS-PNT/01_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/04_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/05_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/06_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/07_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/08_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/09_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/01_1225_Ground_Youyi",
        "/mnt/f/PNT/实验/NMS-PNT/04_0521_Ground_Yunzhi",
        # "/mnt/e/PNT/01",
        # "/mnt/e/PNT/02",
        # "/mnt/e/PNT/03",
        # "/mnt/e/PNT/04",
    ]

    for f_dir in folder_list:
        print("processing item ", f_dir)
        bag_files = [f for f in os.listdir(f_dir) if f.endswith('.bag')]
        bag_file = os.path.join(f_dir, bag_files[0])
        print("found bag file ", bag_file)

        main_euroc_folder(bag_file)


if __name__ == "__main__":
    front_bag_path = "/mnt/f/PNT/实验/NMS-PNT/01_0705_Ground_Wangan/frontend_2025-07-05-18-49-20.bag"
    # back_bag_path = "/mnt/f/PNT/实验/NMS-PNT/09_1225_Ground_Youyi/IGNORED/backend_2024-12-25-16-50-28.bag"
    rtkpos_path = "/mnt/f/PNT/实验/NMS-PNT/01_0705_Ground_Wangan/GNSS/rtk01_0705_Wangan.txt"

    # main_euroc_folder(front_bag_path)
    # main_merge_rtk(front_bag_path, back_bag_path)
    # main_icgvins(front_bag_path, back_bag_path)
    main_icgvins_rtklib(front_bag_path, rtkpos_path)
    # main_merge()
    # main_param_fromat()
    # main_lidar(front_bag_path)
    # main_euroc_folder(front_bag_path)

    # main_lidar(front_bag_path)
