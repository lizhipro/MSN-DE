from evo.main_rpe import rpe
from evo.main_ape import ape
from evo.tools import plot
from evo.core.trajectory import PoseTrajectory3D
from evo.core import metrics, sync
from evo.tools import file_interface
import os
import rospy
import rosbag
import numpy as np
import pandas as pd
# from pnt_msgs.msg import VIO, LIO
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from evo.core import trajectory
import copy
from plot_NMS import NMSPlotter


def read_bag(bag_filename):
    # 初始化ROS节点，虽然在读取bag文件时并不需要真正的ROS节点，但这样可以方便使用rospy的日志功能
    rospy.init_node('bag_reader', anonymous=True)

    xyz = []
    # 打开bag文件
    with rosbag.Bag(bag_filename, 'r') as bag:
        # 遍历bag文件中的每一条消息
        for topic, msg, t in bag.read_messages():
            # 检查消息类型，这里我们假设我们只关心std_msgs/String类型的消息
            # if (topic == "/pose/lidar" or topic == "/pose/cam") and "/byh_uav/ADIS16470" in msgs:
            if topic == "/leica/position":
                # print(msg)
                xyz.append([msg.point.x, msg.point.y, msg.point.z])
    return np.array(xyz)


if __name__ == "__main_old__":
    bag_file = "/mnt/d/Datasets/EuRoC/MH_01_easy.bag"
    # xyz1 = read_bag(bag_file)
    xyz1 = pd.read_csv(
        "src/VINS-Fusion/output/Trajectory_vins_TUM_2_0.txt", header=None)
    xyz = pd.read_csv("src/VINS-Fusion/output/vio_loop.csv", header=None)

    xyz = xyz.values[:, 1:]
    xyz1 = xyz1.values[:, 1:]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2])
    ax.plot(xyz1[:, 0], xyz1[:, 1], xyz1[:, 2])

    print("--- end ---")

    # evo_traj euroc output/vio.csv --ref=output/data.csv -p --plot_mode=xyz -a
    # evo_ape/evo_rpe euroc -a src/VINS-Fusion/output/data.csv vio.tum -vas -r angle_deg
    # evo_ape/evo_rpe euroc -a src/VINS-Fusion/output/data.csv vio.tum -vas


class TrajectoryPlotter(NMSPlotter):
    def __init__(self):
        super().__init__()
        self.mark_list = {
            "GroundTruth": None,
            "FAST-LIO2": "-o",
            "DLO": "-o",
            "ORB-SLAM3(Stereo)": "-D",
            "ORB-SLAM3(Stereo-Inertial)": "-D",
            "VINS(Stereo)": "-D",
            "VINS(Stereo-Inertial)": "-D",

            "VINS(Global-Fusion)": "-s",
            "IC-GVINS": "-s",
            "RTKLIB": "--"
        }

    def plot_trajectories_err(self, axs, traj_by_label: dict, ignored=[]):
        for label, traj in traj_by_label.items():
            if label not in ignored and label != "GroundTruth":
                err = traj.positions_xyz - \
                    traj_by_label['GroundTruth'].positions_xyz
                if label == "RTKLIB":
                    axs[0].plot(traj.timestamps, err[:,
                                0], 'o-', label=label, linewidth=1.5, ms=2.5)
                    axs[1].plot(traj.timestamps, err[:,
                                1], 'o-', label=label, linewidth=1.5, ms=2.5)
                    axs[2].plot(traj.timestamps, err[:,
                                2], 'o-', label=label, linewidth=1.5, ms=2.5)
                else:
                    axs[0].plot(traj.timestamps, err[:,
                                0], label=label, linewidth=2.5)
                    axs[1].plot(traj.timestamps, err[:,
                                1], label=label, linewidth=2.5)
                    axs[2].plot(traj.timestamps, err[:,
                                2], label=label, linewidth=2.5)
            else:
                axs[0].plot([], [], linewidth=2.5)
                axs[1].plot([], [], linewidth=2.5)
                axs[2].plot([], [], linewidth=2.5)
            axs[0].legend(loc='upper left', framealpha=0.3)
            # axs[0].set_aspect('equal')
            axs[0].tick_params(direction='in')
            axs[0].set_ylabel("x/m")
            axs[1].set_ylabel("y/m")
            axs[2].set_ylabel("z/m")
            axs[2].set_xlabel("seconds of week/s")
        return

    def plot_trajectories(self, axs, traj_by_label: dict, ignored=[]):
        # plt.subplots_adjust(
        #     top=0.973,
        #     bottom=0.083,
        #     left=0.165,
        #     right=0.952,
        #     hspace=0.175,
        #     wspace=0.2
        # )
        for label, traj in traj_by_label.items():
            if label not in ignored:
                if label == "RTKLIB":
                    axs[0].plot(traj.positions_xyz[:, 0],
                                traj.positions_xyz[:, 1], '--', label=label, linewidth=2.0)

                    axs[1].plot(traj.timestamps, traj.positions_xyz[:, 2], '--',
                                label=label, linewidth=1.5)
                elif label == "GroundTruth":
                    axs[0].plot(traj.positions_xyz[:, 0],
                                traj.positions_xyz[:, 1], label=label, linewidth=2.0)
                    axs[1].plot(traj.timestamps, traj.positions_xyz[:, 2],
                                label=label, linewidth=2)
                else:
                    axs[0].plot(traj.positions_xyz[:, 0],
                                traj.positions_xyz[:, 1], self.mark_list[label], markevery=10, label=label, linewidth=1.5, ms=3.5)
                    axs[1].plot(traj.timestamps, traj.positions_xyz[:, 2],
                                self.mark_list[label], markevery=10, label=label, linewidth=1.5, ms=3.5)
            else:
                # axs[0].plot([], [], label='_nolegend_', linewidth=2.0)
                # axs[1].plot([], [], label='_nolegend_', linewidth=2.0)
                axs[0].plot([], [], self.mark_list[label],
                            markevery=10, label=label, linewidth=1.5, ms=3.5)
                axs[1].plot([], [], self.mark_list[label],
                            markevery=10, label=label, linewidth=1.5, ms=3.5)

        axs[0].set_xlabel("x/m")
        axs[0].set_ylabel("y/m")
        axs[1].set_xlabel("seconds of week/s")
        axs[1].set_ylabel("z/m")
        return


def evo_process(dataset_dir, ignored=[], traj_config={}, save_fig=False, n_align=120):

    traj_with_label = {}

    df_gt = pd.read_csv(os.path.join(
        dataset_dir, traj_config['GroundTruth']), header=None, sep='\s+')
    df_gt.columns = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']

    for name, tj_file in traj_config.items():
        if name in ignored:
            traj_with_label[name] = None
            continue
        tj = os.path.join(dataset_dir, tj_file)
        traj_with_label[name] = file_interface.read_tum_trajectory_file(tj)

    ts_end = df_gt['timestamp'].values[0]
    ts_start = df_gt['timestamp'].values[-1]

    # 计算APE
    APEs = ""
    methods = ""
    ape_metric_trans = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric_rot = metrics.APE(metrics.PoseRelation.rotation_angle_rad)
    rpe_metric_trans = metrics.RPE(
        metrics.PoseRelation.translation_part, delta=1)
    rpe_metric_rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_rad, delta=1)
    df_res = pd.DataFrame()

    for name, traj in traj_with_label.items():
        if name == "GroundTruth":
            continue
        if name in ignored:
            methods = methods + name + " | "
            APEs = APEs + "& - & - "
            continue
        if "ORB" in name:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.025)
            traj_with_label[name].align(
                traj_gt, correct_scale=False, n=n_align)
        elif "RTK" in name:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.5)
            traj_with_label[name].align(
                traj_gt, correct_scale=False, n=n_align)
        else:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.1)
            traj_with_label[name].align(
                traj_gt, correct_scale=False, n=n_align)

        ts_end = max(ts_end, traj_gt.timestamps[-1])
        ts_start = min(ts_start, traj_gt.timestamps[0])

        ape_metric_trans.process_data((traj_gt, traj_with_label[name]))
        ape_metric_rot.process_data((traj_gt, traj_with_label[name]))
        rpe_metric_trans.process_data((traj_gt, traj_with_label[name]))
        rpe_metric_rot.process_data((traj_gt, traj_with_label[name]))
        df_curr = pd.DataFrame(
            {"timestamp": traj_gt.timestamps,
             name+"_ape": ape_metric_trans.error,
             name+"_ape_rot": ape_metric_rot.error})
        df_curr[[name+"_rpe_trans", name+"_rpe_rot"]] = np.nan
        df_curr[name+"_rpe_trans"][rpe_metric_trans.delta_ids] = rpe_metric_trans.error
        df_curr[name+"_rpe_rot"][rpe_metric_rot.delta_ids] = rpe_metric_rot.error

        if df_res.empty:
            df_res = df_curr
        else:
            df_res = pd.merge(df_res, df_curr, on="timestamp",
                              how="outer").sort_values("timestamp")
        ape_stats = ape_metric_trans.get_all_statistics()
        methods = methods + name + " | "
        APEs = APEs + "& %.3f " % ape_stats['rmse'] + "& %.3f " % (ape_stats['rmse'] /
                                                                   traj_gt.path_length * 100)

    # load进dataframe里，和每个轨迹associate之后获得起止时间
    # 截取dataframe的tx,ty,tz,qx,qy,qz,qw，重新生成GroundTruth的traj
    df_gt = df_gt[(df_gt['timestamp'] > ts_start)
                  & (df_gt['timestamp'] < ts_end)]

    traj_with_label["GroundTruth"] = PoseTrajectory3D(
        df_gt[['tx', 'ty', 'tz']].values, df_gt[['qx', 'qy', 'qz', 'qw']].values, df_gt['timestamp'].values)

    df_res = pd.merge(df_res, df_gt[["timestamp"]], on="timestamp",
                      how="outer").sort_values("timestamp")

    # plot
    traj_plotter = TrajectoryPlotter()
    fig, axs = plt.subplots(2, 1, figsize=(6, 8), gridspec_kw={
        'height_ratios': [1.6, 1]})
    traj_plotter.plot_trajectories(
        axs, traj_with_label, ignored=ignored)
    # plt.tight_layout()
    plt.subplots_adjust(
        top=0.97,
        bottom=0.093,
        left=0.144,
        right=0.96,
        hspace=0.237,
        wspace=0.165
    )

    # axs[0].legend(loc='upper right', framealpha=0.0, ncols=1)
    # axs[0].set_aspect('equal')
    # axs[0].set_ylim(-100, 150)
    # axs[1].set_ylim(-40, 25)
    # axs[1].set_ylim(-6, 5)
    if save_fig:
        plt.savefig(os.path.join(dataset_dir, "%s.pdf" % os.path.basename(dataset_dir)),
                    dpi=400, bbox_inches='tight')

    err_cols = [item for item in list(traj_with_label.keys())[
        1:] if item not in ignored]
    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    axs[0].plot([], [])
    axs[0].plot(df_res['timestamp'], df_res[[col+"_ape" for col in err_cols]],
                'o-', label=err_cols, linewidth=1.5, ms=2.5)
    axs[1].plot([], [])
    axs[1].plot(df_res['timestamp'], df_res[[col+"_ape_rot" for col in err_cols]],
                'o-', label=err_cols, linewidth=1.5, ms=2.5)

    axs[2].plot([], [])
    axs[2].plot(df_res['timestamp'], df_res[[col+"_rpe_trans" for col in err_cols]],
                'o-', label=err_cols, linewidth=1.5, ms=2.5)
    axs[3].plot([], [])
    axs[3].plot(df_res['timestamp'], df_res[[col+"_rpe_rot" for col in err_cols]],
                'o-', label=err_cols, linewidth=1.5, ms=2.5)
    axs[0].legend()
    axs[0].set_ylabel("APE of Translation/m")
    axs[1].set_ylabel("APE of Rotation/rad")
    axs[2].set_ylabel("RPE of Translation/m")
    axs[3].set_ylabel("RPE of Rotation/rad")
    axs[-1].set_xlabel("seconds of week/s")

    plt.tight_layout()

    # output
    print("======= " + os.path.basename(dataset_dir) + " =======")

    print("Total trajectory length: %dposes, %.3fm, %.3fs" %
          (len(traj_with_label["GroundTruth"].timestamps),
           traj_with_label["GroundTruth"].path_length,
           traj_with_label["GroundTruth"].timestamps[-1] - traj_with_label["GroundTruth"].timestamps[0]))
    print(methods)
    print(APEs)

    df_res.to_csv(os.path.join(dataset_dir, "evo_ape.csv"), index=None)


def evo_dataset(dataset_dir, ignored=[], traj_config={}, save_fig=False):

    traj_with_label = {}

    df_gt = pd.read_csv(os.path.join(
        dataset_dir, traj_config['GroundTruth']), header=None, sep='\s+')
    df_gt.columns = ['#timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']

    for name, tj_file in traj_config.items():
        # if name in ignored:
        #     traj_with_label[name] = None
        #     continue
        tj = os.path.join(dataset_dir, tj_file)
        traj_with_label[name] = file_interface.read_tum_trajectory_file(tj)

    ts_end = df_gt['#timestamp'].values[0]
    ts_start = df_gt['#timestamp'].values[-1]
    for name, traj in traj_with_label.items():
        if name == "GroundTruth":
            continue
        # if name in ignored:
        #     continue
        if "ORB" in name:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.025)
            traj_with_label[name].align(traj_gt, correct_scale=False, n=120)

        elif "RTK" in name:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.5)
            traj_with_label[name].align(traj_gt, correct_scale=False, n=120)
        else:
            traj_gt, traj_with_label[name] = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.05)
            traj_with_label[name].align(
                traj_gt, correct_scale=False, n=120)

        ts_end = max(ts_end, traj_gt.timestamps[-1])
        ts_start = min(ts_start, traj_gt.timestamps[0])

    # orb_slam结果频率最高，把orb_slam结果放在最后边，associate之后是最全且剪切后的GT
    df_gt = df_gt[(df_gt['#timestamp'] > ts_start)
                  & (df_gt['#timestamp'] < ts_end)]

    traj_with_label["GroundTruth"] = PoseTrajectory3D(
        df_gt[['tx', 'ty', 'tz']].values, df_gt[['qx', 'qy', 'qz', 'qw']].values, df_gt['#timestamp'].values)

    traj_plotter = TrajectoryPlotter()

    fig, axs1 = plt.subplots(2, 1, figsize=(7, 10), gridspec_kw={
        'height_ratios': [1.6, 1]})
    traj_plotter.plot_trajectories(
        axs1, traj_with_label, ignored=ignored)
    plt.tight_layout()

    # fig, axs2 = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    # traj_plotter.plot_trajectories_err(
    #     axs2, traj_with_label, ignored=ignored)
    # plt.tight_layout()
    # axs2[1].set_xlim([374871, 375235])

    if save_fig:
        plt.savefig(os.path.join(dataset_dir, "%s.pdf" % os.path.basename(dataset_dir)),
                    dpi=400, bbox_inches='tight')
    # plt.show()

    # 计算APE
    APEs = ""
    methods = ""
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    df_res = pd.DataFrame()
    for name, traj in traj_with_label.items():
        if name == "GroundTruth":
            print("Total trajectory length: %dposes, %.3fm, %.3fs" %
                  (len(traj.timestamps), traj.path_length, traj.timestamps[-1] - traj.timestamps[0]))
            df_res["timestamp"] = traj.timestamps
            continue
        if name in ignored:
            methods = methods + name + " | "
            APEs = APEs + "& - & - "
            df_res[name+"_ape"] = np.nan
            continue
        if "ORB" in name:
            traj_gt, traj_est = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.025)
        elif "RTK" in name:
            traj_gt, traj_est = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.5)
        else:
            traj_gt, traj_est = sync.associate_trajectories(
                traj_with_label["GroundTruth"], traj_with_label[name], max_diff=0.05)
        traj_est.align(traj_gt, correct_scale=False, n=300)
        ape_metric.process_data((traj_gt, traj_est))
        df_curr = pd.DataFrame(
            {name+"_ape": ape_metric.error, "timestamp": traj_gt.timestamps})
        df_res = pd.merge(df_res, df_curr, on="timestamp",
                          how="outer").sort_values("timestamp")
        ape_stats = ape_metric.get_all_statistics()
        methods = methods + name + " | "
        APEs = APEs + "& %.3f " % ape_stats['rmse'] + "& %.3f " % (ape_stats['rmse'] /
                                                                   traj_with_label["GroundTruth"].path_length * 100)
        # APEs.append(ape_stats['rmse'], ape_stats['rmse'] /
        #             traj_with_label["GroundTruth"].path_length * 100)
    print(methods)
    print(APEs)

    fig, axs = plt.subplots(figsize=(10, 5))
    axs.plot([], [])
    axs.plot(df_res['timestamp'], df_res[df_res.columns[1:]],
             'o-', label=list(traj_with_label.keys())[1:], linewidth=1.5, ms=2.5)
    axs.legend()
    axs.set_xlabel("seconds of week/s")
    axs.set_ylabel("APE/m")
    plt.tight_layout()

    df_res.to_csv(os.path.join(dataset_dir, "evo_error.csv"), index=None)

    if save_fig:
        plt.savefig(os.path.join(dataset_dir, "%s.pdf" % os.path.basename(dataset_dir)),
                    dpi=400, bbox_inches='tight')
    plt.show()


def main():
    traj_config3 = {
        "GroundTruth": "GroundTruth.TXT",
        "FAST-LIO2": "Trajectory_fast-lio_TUM.txt",
        "DLO": "Trajectory_dlo_TUM.txt",
        "ORB-SLAM3(Stereo)": "Trajectory_orb_TUM_stereo.txt",
        "ORB-SLAM3(Stereo-Inertial)": "Trajectory_orb_TUM_stereo_inertial.txt",
        "VINS(Stereo)": "Trajectory_vins_TUM_2_0_loop.txt",
        "VINS(Stereo-Inertial)": "Trajectory_vins_TUM_2_1_loop.txt",

        "VINS(Global-Fusion)": "Trajectory_vins_TUM_global.txt",
        "IC-GVINS": "Trajectory_icgvins_TUM.txt",
        "RTKLIB": "Trajectory_rtk_TUM.txt"
    }
    traj_config_gt = {
        "GroundTruth": "GroundTruth.TXT",
        # "FAST-LIO2": "Trajectory_fast-lio_TUM.txt",
    }
    # ignored_paths = ["FAST-LIO2", "DLO",
    #                  "ORB-SLAM3(Stereo)", "ORB-SLAM3(Stereo-Inertial)"]
    # indoor_ignored_paths = ["RTKLIB"]
    # rtk_ignored_paths = ["RTKLIB"]
    rtk_ignored_paths = [
        "FAST-LIO2",
        "DLO",
        "VINS(Stereo)",
        # "VINS(Stereo-Inertial)",
        "ORB-SLAM3(Stereo)",
        # "ORB-SLAM3(Stereo-Inertial)",
        # "IC-GVINS",
        # "VINS(Global-Fusion)",
        # "RTKLIB",
    ]

    dataset_list = [
        # "/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/06_1128_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/09_1225_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/01_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_1208_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/03_1209_Ground_Yunzhi",
        "/mnt/f/PNT/实验/NMS-PNT/05_0521_Ground_Yunzhi",
        # "/mnt/f/PNT/实验/NMS-PNT/02_0705_Ground_Youyi",
        # "/mnt/f/PNT/实验/NMS-PNT/01_0705_Ground_Wangan",
        # "/mnt/f/PNT/实验/NMS-PNT/03_0702_Ground_Wangan"
    ]

    for dataset_dir in dataset_list:
        evo_process(dataset_dir, ignored=rtk_ignored_paths,
                    traj_config=traj_config3, n_align=120)

    print("--- end ---")


if __name__ == "__main__":
    main()


# evo_traj tum /mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi/Trajectory_vins_TUM_2_1.txt --ref=/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi/Trajectory_vins_TUM_1_1.txt -p --plot_mode=xz
