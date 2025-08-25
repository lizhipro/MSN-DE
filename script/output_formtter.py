#!/usr/bin/env python

import os
import rospy
import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
import pandas as pd
from scipy.spatial.transform import Rotation as R
from gps_time import GPSTime
from datetime import datetime, timedelta

from sensor_msgs.msg import Image
# from byh_uav.msg import uav_imu
from tqdm import tqdm
import numpy as np
import pymap3d as pm
import simplekml

bridge = CvBridge()


def gt2tum(gt_path, heads=26, footers=-3):

    columns = [
        "GPSTime", "Latitude", "Longitude", "H-Ell", "X-ECEF", "Y-ECEF", "Z-ECEF", "Heading", "Pitch", "Roll"
    ]
    columns = [
        "GPSTime", "Latitude", "Longitude", "H-Ell", "Heading", "Pitch", "Roll"
    ]
    df_gt = pd.read_csv(gt_path, skiprows=heads, header=None, sep="\s+")
    # df_gt = df_gt[[1, 2, 3, 4, 9, 10, 11, 21, 22, 23]].iloc[:footers, :]
    df_gt = df_gt[[1, 2, 3, 4, 6, 7, 8]].iloc[:footers, :]
    df_gt.columns = columns
    euler_angles_deg = df_gt[["Heading", "Pitch", "Roll"]].values
    euler_angles_rad = np.deg2rad(euler_angles_deg)
    rot = R.from_euler('ZYX', euler_angles_rad)
    quats = rot.as_quat()
    df_gt[["qx", "qy", "qz", "qw"]] = quats

    # ENU 原点（参考点，通常是第一个点的LLH）
    lat0 = float(df_gt["Latitude"][0])  # degrees
    lon0 = float(df_gt["Longitude"][0])
    h0 = float(df_gt["H-Ell"][0])  # meters

    # 批量转换
    # lon, lat, h = pm.ecef2geodetic(df.iloc[:, 8], df.iloc[:, 9], df.iloc[:, 10])
    # e, n, u = pm.ecef2enu(df_gt["X-ECEF"].values.astype(float), df_gt["Y-ECEF"].values.astype(
    #     float), df_gt["Z-ECEF"].values.astype(float), lat0, lon0, h0)
    e, n, u = pm.geodetic2enu(
        df_gt["Latitude"], df_gt["Longitude"], df_gt["H-Ell"], lat0, lon0, h0)
    df_gt['E'] = e
    df_gt['N'] = n
    df_gt['U'] = u
    df_export = df_gt[["GPSTime", "E", "N", "U", "qx", "qy", "qz", "qw"]]

    df_export.to_csv(gt_path, index=None, header=None, sep=' ')


def rtk2tum(bag_file, export_dir):

    rtk_msgs = []
    with rosbag.Bag(bag_file, 'r') as bag:
        # 遍历bag文件中的每一条消息
        for topic, msg, t in tqdm(bag.read_messages()):
            if topic == "/pose/gnss":
                rtk_msgs.append([msg.timestamp,
                                msg.latitude, msg.longitude, msg.height])
    lon0 = rtk_msgs[0][2]
    lat0 = rtk_msgs[0][1]
    h0 = rtk_msgs[0][3]

    rtk_msgs = np.array(rtk_msgs)

    e, n, u = pm.geodetic2enu(rtk_msgs[:, 1], rtk_msgs[:, 2],
                              rtk_msgs[:, 3], lat0, lon0, h0)
    df_rtk = pd.DataFrame({
        "timestamp": rtk_msgs[:, 0],
        "E": e,
        "N": n,
        "U": u,
        "qx": 0,
        "qy": 0,
        "qz": 0,
        "qw": 0
    })
    df_rtk.to_csv(os.path.join(export_dir, "Trajectory_rtk_TUM.txt"),
                  index=None, header=None, sep=' ')


def rtkpos2tum(rtkpos_path, export_dir):

    df_rtk = pd.read_csv(rtkpos_path, sep='\s+')
    df_rtk['datetime_str'] = df_rtk['%'] + ' ' + df_rtk['GPST']
    df_rtk['datetime'] = pd.to_datetime(
        df_rtk['datetime_str'], format='%Y/%m/%d %H:%M:%S.%f')
    df_rtk["datetime"] = df_rtk["datetime"].apply(
        lambda dt: GPSTime.from_datetime(dt).time_of_week)

    lon0 = df_rtk["longitude(deg)"][0]
    lat0 = df_rtk["latitude(deg)"][0]
    h0 = df_rtk["height(m)"][0]

    e, n, u = pm.geodetic2enu(df_rtk["latitude(deg)"].values, df_rtk["longitude(deg)"].values,
                              df_rtk["height(m)"].values, lat0, lon0, h0)
    df_rtk = pd.DataFrame({
        "timestamp": df_rtk["datetime"],
        "E": e,
        "N": n,
        "U": u,
        "qx": 0,
        "qy": 0,
        "qz": 0,
        "qw": 0
    })
    df_rtk.to_csv(os.path.join(export_dir, "Trajectory_rtk_TUM.txt"),
                  index=None, header=None, sep=' ')


def vins2tum(vins_path):

    df_data = pd.read_csv(vins_path, header=None)

    if len(df_data.columns) == 8:
        if "fast-lio" in vins_path:
            df_data[0] = df_data[0].apply(lambda dt: GPSTime.from_datetime(
                datetime.fromtimestamp(dt*1e-9) - timedelta(hours=8, seconds=-18)).time_of_week)
        else:
            df_data[0] = 1e-9 * df_data[0]

        df_data = df_data[[0, 1, 2, 3, 5, 6, 7, 4]]
        df_data.to_csv(vins_path, index=None, header=None,
                       sep=' ', float_format="%.5f")


def vicon2tum(bag_file, export_dir):
    vicon_msgs = []

    with rosbag.Bag(bag_file, 'r') as bag:
        gpst = -1
        # 遍历bag文件中的每一条消息
        for topic, msg, t in tqdm(bag.read_messages()):
            if topic == "/byh_uav/ADIS16470":
                gpst = msg.accel_gps_time
            if topic == "/vicon/PNT_ANT/PNT_ANT" and gpst > 0:
                vicon_msgs.append([gpst,
                                   msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z,
                                   msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])
        vicon_msgs = np.array(vicon_msgs)
        np.savetxt(os.path.join(export_dir, "GroundTruth.TXT"),
                   vicon_msgs, fmt="%.5f")


def csv2kml(csv_file):
    df = pd.read_csv(csv_file)
    kml = simplekml.Kml()

    pnt = kml.newpoint(coords=[(df.iloc[0]['lon'], df.iloc[0]['lat'])])
    pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/blu-blank.png"
    pnt.style.iconstyle.color = "ff2ca02c"
    pnt.style.iconstyle.scale = 0.8

    pnt = kml.newpoint(coords=[(df.iloc[-1]['lon'], df.iloc[-1]['lat'])])
    pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/paddle/blu-blank.png"
    pnt.style.iconstyle.color = "ff2827D6"
    pnt.style.iconstyle.scale = 0.8

    for i in range(1, len(df)):
        lin = kml.newlinestring(coords=[
                                (df.iloc[i]['lon'], df.iloc[i]['lat']), (df.iloc[i-1]['lon'], df.iloc[i-1]['lat'])])
        lin.style.linestyle.color = 'ffd9d9d9'
        lin.style.linestyle.width = 2

        pnt = kml.newpoint(coords=[(df.iloc[i]['lon'], df.iloc[i]['lat'])])
        pnt.style.iconstyle.icon.href = "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png"
        pnt.style.iconstyle.color = "ff0E7FFF"
        pnt.style.iconstyle.scale = 0.8

    kml.save(os.path.join(os.path.dirname(csv_file),
             os.path.basename(csv_file) + ".kml"))


def combine_mocap_100c(path_mocap, path_100c, ts_split):
    columns = [
        "GPSTime", "Latitude", "Longitude", "H-Ell", "X-ECEF", "Y-ECEF", "Z-ECEF", "Heading", "Pitch", "Roll"
    ]
    df_gt = pd.read_csv(path_100c, skiprows=26, header=None, sep="\s+")
    df_gt = df_gt[[1, 2, 3, 4, 9, 10, 11, 21, 22, 23]].iloc[:-3, :]
    df_gt.columns = columns
    euler_angles_deg = df_gt[["Heading", "Pitch", "Roll"]].values
    euler_angles_rad = np.deg2rad(euler_angles_deg)
    rot = R.from_euler('ZYX', euler_angles_rad)
    quats = rot.as_quat()
    df_gt[["qx", "qy", "qz", "qw"]] = quats

    # ENU 原点（参考点，通常是第一个点的LLH）
    lat0 = float(df_gt["Latitude"][0])  # degrees
    lon0 = float(df_gt["Longitude"][0])
    h0 = float(df_gt["H-Ell"][0])  # meters

    # 批量转换
    # lon, lat, h = pm.ecef2geodetic(df.iloc[:, 8], df.iloc[:, 9], df.iloc[:, 10])
    e, n, u = pm.ecef2enu(df_gt["X-ECEF"].values.astype(float), df_gt["Y-ECEF"].values.astype(
        float), df_gt["Z-ECEF"].values.astype(float), lat0, lon0, h0)
    df_gt['E'] = e
    df_gt['N'] = n
    df_gt['U'] = u
    df_gt["GPSTime"] = df_gt["GPSTime"].astype(float)
    df_100c_export = df_gt[["GPSTime", "E", "N", "U", "qx", "qy", "qz", "qw"]]
    df_100c_export = df_100c_export[df_100c_export['GPSTime'] % 1 == 0]

    td = np.diff(df_100c_export['GPSTime'].values)
    if np.sum(td) != td.size:
        print("timestamp with jump", np.sum(td)-td.size())

    df_100c_export.to_csv(os.path.join(os.path.dirname(path_100c),
                          "GroundTruth.TXT"), header=None, index=None, sep=' ')

    df_vicon = pd.read_csv(path_mocap)
    df_vicon_export = df_vicon[["field.header.stamp",
                                'field.transform.translation.x', 'field.transform.translation.y', 'field.transform.translation.z',
                                "field.transform.rotation.x", "field.transform.rotation.y", "field.transform.rotation.z", "field.transform.rotation.w"]]
    df_vicon_export["time"] = df_vicon_export["field.header.stamp"].apply(lambda dt: GPSTime.from_datetime(
        datetime.fromtimestamp(dt*1e-9) - timedelta(hours=8, seconds=-18)).time_of_week)

    xyz_vicon = df_vicon_export[['field.transform.translation.x',
                                 'field.transform.translation.y', 'field.transform.translation.z']].values

    r_ecef_vicon = np.array([[-0.8925911,  0.0775095, -0.4441547],
                             [-0.41111654, -0.54436243,  0.73119952],
                             [-0.18510623,  0.83526153,  0.5177585]])
    t_ecef_vicon = np.array(
        [-2.840642602193585e6, 4.673149112115148e6, 3.271256068543798e6])

    xyz_exef = np.dot(r_ecef_vicon, xyz_vicon.transpose()
                      ).transpose() + t_ecef_vicon
    e, n, u = pm.ecef2enu(
        xyz_exef[:, 0], xyz_exef[:, 1], xyz_exef[:, 2], lat0, lon0, h0)

    df_vicon_export["e"] = e
    df_vicon_export["n"] = n
    df_vicon_export["u"] = u

    print("=== End ===")


if __name__ == "__main_gt__":
    data_dir = "/mnt/f/PNT/实验/NMS-PNT/09_1225_Ground_Youyi"
    gt_path = os.path.join(data_dir, "GroundTruth.TXT")

    gt2tum("/mnt/f/PNT/实验/NMS-PNT/03_0702_Ground_Wangan/GroundTruth.TXT", heads=2)

    # vins_paths = [f for f in os.listdir(data_dir)
    #               if ("vins" in f or "fast-lio" in f) and os.path.isfile(os.path.join(data_dir, f))]
    # for vins_path in vins_paths:
    #     vins2tum(os.path.join(data_dir, vins_path))
    print("---end---")

if __name__ == "__main__":
    bag_file = [
        # "/mnt/f/PNT/实验/1128/backend_2024-11-28-15-30-46.bag",
        # "/mnt/f/PNT/实验/1128/backend_2024-11-28-15-46-44.bag",
        # "/mnt/f/PNT/实验/1128/backend_2024-11-28-16-03-26.bag",
        # "/mnt/f/PNT/实验/1128/backend_2024-11-28-16-36-01.bag",
        "/mnt/f/PNT/实验/1225/backend_2024-12-25-16-50-28.bag",

        # "/mnt/f/PNT/实验/1208/backend_2024-12-08-19-42-28.bag",
        # "/mnt/f/PNT/实验/1208/backend_2024-12-08-19-57-36.bag",
        # "/mnt/f/PNT/实验/1209/backend_2024-12-09-11-05-24.bag",
        # "/mnt/f/PNT/实验/1209/backend_2024-12-09-11-17-14.bag",
        # "/mnt/f/PNT/实验/1209/backend_2024-12-09-11-28-14.bag"
    ]
    export_dir = [
        # "/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/05_1128_Ground_Youyi",
        "/mnt/f/PNT/实验/NMS-PNT/01_1225_Ground_Youyi",
        "/mnt/f/PNT/实验/NMS-PNT/01_1225_Ground_Youyi",

        # "/mnt/f/PNT/实验/NMS-PNT/01_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/04_1209_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/05_1209_Ground_Yunzhi"
    ]

    rtkpos2tum("/mnt/f/PNT/实验/NMS-PNT/03_0702_Ground_Wangan/GNSS/rtk03_0702_Wangan.txt",
               "/mnt/f/PNT/实验/NMS-PNT/03_0702_Ground_Wangan")
    # rtk2tum(bag_file, export_dir)
    # for bag, ex in zip(bag_file, export_dir):
    # vicon2tum(bag, ex)
    # rtk2tum(bag, ex)
    # vins_paths = [f for f in os.listdir(ex)
    #               if ("vins" in f or "fast-lio" in f) and os.path.isfile(os.path.join(ex, f))]
    # for vins_path in vins_paths:
    #     vins2tum(os.path.join(ex, vins_path))


def tum_tmp(file_p):
    df = pd.read_csv(file_p, sep="\s+", header=None)
    df = df[::200]
    lat0 = 31.05669314753437
    lon0 = 121.29398524707999
    h0 = 14.899777930123141
    e, n, u = pm.ecef2enu(df[8].values, df[9].values,
                          df[10].values, lat0, lon0, h0)
    df_rtk = pd.DataFrame({
        "timestamp": df[0].values,
        "E": e,
        "N": n,
        "U": u,
        "qx": 0,
        "qy": 0,
        "qz": 0,
        "qw": 0
    })
    df_rtk.to_csv(os.path.join(os.path.dirname(file_p), "Trajectory_filterW_TUM.txt"),
                  index=None, header=None, sep=' ')


if __name__ == "__main_100__":

    # csv2kml("/mnt/d/Users/liban/Downloads/TRUTH.csv")
    # file_p = "/mnt/f/PNT/实验/0521/03/result_20250521_123106.txt"
    # t,lat,lon,height
    # tum_tmp(file_p)
    # csv2kml("/mnt/d/Pride/重点研发计划/数据集论文/manuscript/src/youyi_gt.csv")

    # rtk2tum("/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi/IGNORED/backend_2025-05-21-12-06-51.bag",
    #         "/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi")

    combine_mocap_100c(
        "/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi/2025-05-21-12-03-37.csv",
        "/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi/天线相位中心.ref",
        137.5
    )

    print("--- end ---")
