from pathlib import Path
import matplotlib.dates as mdates
from matplotlib.dates import DateFormatter
import georinex as gr
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.stats import gaussian_kde
from gps_time import GPSTime
from datetime import datetime, timedelta
import os


class NMSPlotter:
    def __init__(self):
        plt.style.use('default')
        plt.rcParams['font.family'] = 'Arial'
        plt.rcParams['font.size'] = 16
        plt.rcParams['xtick.direction'] = 'in'
        plt.rcParams['ytick.direction'] = 'in'
        plt.rcParams['grid.color'] = 'lightgray'
        self.custom_colors = ['#000000', '#1f77b4',
                              '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8C564B', '#E377C2', '#BCBD22', '#4DBEEE', '#17BECF']
        plt.rcParams['axes.grid'] = True
        from cycler import cycler
        plt.rcParams['axes.prop_cycle'] = cycler(color=self.custom_colors)

    def plot_rinex(self, axs, df_rinex: pd.DataFrame, time_range=None, save_fig=False):
        if time_range is not None and time_range[1] > time_range[0]:
            df_rinex = df_rinex[(df_rinex["Timestamp"] > time_range[0]) & (
                df_rinex["Timestamp"] < time_range[1])]
        axs[0].plot([], [])
        axs[0].plot(df_rinex["Timestamp"], df_rinex["GPS_Avg_CN0"],
                    label="GPS", linewidth=2.5)
        axs[0].plot(df_rinex["Timestamp"], df_rinex["BDS_Avg_CN0"],
                    label="Beidou", linewidth=2.5)
        axs[0].plot(df_rinex["Timestamp"], df_rinex["Galileo_Avg_CN0"],
                    label="Galileo", linewidth=2.5)
        axs[1].step(df_rinex["Timestamp"], df_rinex["Total_Count"],
                    label="Num of satellites", linewidth=2.5)
        axs[0].legend(loc="lower left", framealpha=0.0)
        axs[0].set_ylabel("Avg. CN0/dB-HZ")
        axs[-1].set_xlabel("seconds of week/s")
        axs[1].set_ylabel("Num of satellites")
        plt.tight_layout()

    def plot_timestamp(self, axs, df, label):
        data = np.diff(
            df[["field.header.stamp", "field.accel_gps_time"]].values, axis=0)
        for i in range(data.shape[-1]):
            counts, bins = np.histogram(
                data[:, i], bins=np.arange(0, 0.010, 0.0005))
            percent = counts / counts.sum() * 100
            width = bins[1] - bins[0]

            axs[i].bar(bins[:-1], percent, width=width,
                       align='edge', color=self.custom_colors[i+1], alpha=0.7)
            axs[i].set_ylabel("Percentage (%)")

        axs[0].legend(['System timestamp'], loc="top right")
        axs[1].legend(['GNSS timestamp'], loc="top right")

        axs[-1].set_xlabel("Sampling interval of %s (s)" % label)
        # axs.legend()

        plt.tight_layout()

        # kde = gaussian_kde(data)
        # x_vals = np.linspace(min(data), max(data), 1000)
        # ax.plot(x_vals, kde(x_vals), color='red', linewidth=2)


def analyze_multi_gnss(file_path):
    """
    统计 RINEX 文件中每个历元的 GPS/BDS/Galileo 卫星数量和平均载噪比

    参数:
        file_path (str): RINEX 文件路径

    返回:
        pd.DataFrame: 包含时间戳和各系统统计结果的数据框
    """
    # 定义各系统信号和载噪比类型
    systems_config = {
        'GPS': {
            'signal': 'C1C',     # GPS L1 C/A 伪距
            'cn0': 'S1C',        # GPS L1 C/A 载噪比
            'sat_prefix': 'G'     # GPS 卫星前缀
        },
        'BDS': {
            'signal': 'C2I',     # BDS B1I 伪距
            'cn0': 'S2I',        # BDS B1I 载噪比
            'sat_prefix': 'C'     # BDS 卫星前缀
        },
        'Galileo': {
            'signal': 'C1X',     # Galileo E1 伪距
            'cn0': 'S1X',        # Galileo E1 载噪比
            'sat_prefix': 'E'     # Galileo 卫星前缀
        }
    }

    # 收集所有需要的观测类型
    required_obs = set()
    for sys in systems_config.values():
        required_obs.add(sys['signal'])
        required_obs.add(sys['cn0'])

    # 读取 RINEX 文件
    try:
        print(f"正在读取文件: {file_path}")
        obs_data = gr.load(file_path, meas=list(required_obs), verbose=False)
        print("文件读取成功!")
    except Exception as e:
        raise IOError(f"读取文件失败: {str(e)}")

    # 获取时间和卫星列表
    times = obs_data.time.values
    sats = obs_data.sv.values

    # 初始化结果数组
    results = {
        'Timestamp': times
    }

    # 为每个系统添加计数和载噪比数组
    for sys_name, config in systems_config.items():
        results[f'{sys_name}_Count'] = np.zeros(len(times), dtype=int)
        results[f'{sys_name}_Avg_CN0'] = np.full(len(times), np.nan)

    # 按历元处理数据
    for i, time in enumerate(times):
        for sys_name, config in systems_config.items():
            # 筛选当前系统的卫星
            sys_mask = [sv.startswith(config['sat_prefix']) for sv in sats]

            # 获取当前系统的信号数据
            signal_data = obs_data[config['signal']].sel(time=time)[sys_mask]

            # 统计有效卫星数
            valid_sats = np.sum(~np.isnan(signal_data))
            results[f'{sys_name}_Count'][i] = valid_sats

            # 如果有有效卫星，计算平均载噪比
            if valid_sats > 0:
                cn0_data = obs_data[config['cn0']].sel(time=time)[sys_mask]
                valid_cn0 = cn0_data[~np.isnan(cn0_data)]
                if len(valid_cn0) > 0:
                    results[f'{sys_name}_Avg_CN0'][i] = np.mean(valid_cn0)

    # 添加总卫星数列
    results['Total_Count'] = np.zeros(len(times), dtype=int)
    for sys_name in systems_config:
        results['Total_Count'] += results[f'{sys_name}_Count']

    # 创建 DataFrame
    df = pd.DataFrame(results)

    # 按时间排序
    df.sort_values('Timestamp', inplace=True)

    df["Timestamp"] = df["Timestamp"].apply(
        lambda dt: GPSTime.from_datetime(dt).time_of_week)

    return df


def save_and_summarize(df, output_file):
    """保存结果并生成统计摘要"""
    # 保存到 CSV
    df.to_csv(output_file, index=False)
    print(f"结果已保存至: {output_file}")

    # 生成统计摘要
    summary = {
        'Total_Epochs': len(df),
        'Average_Total_Sats': df['Total_Count'].mean()
    }

    for sys in ['GPS', 'BDS', 'Galileo']:
        count_col = f'{sys}_Count'
        cn0_col = f'{sys}_Avg_CN0'

        summary[f'{sys}_Avg_Count'] = df[count_col].mean()
        summary[f'{sys}_Max_Count'] = df[count_col].max()
        summary[f'{sys}_Min_Count'] = df[count_col].min()
        summary[f'{sys}_Avg_CN0'] = df[cn0_col].mean()
        summary[f'{sys}_Data_Percentage'] = (df[count_col] > 0).mean() * 100

    # 创建摘要数据框
    summary_df = pd.DataFrame([summary])

    # 保存摘要
    summary_file = output_file.replace('.csv', '_summary.csv')
    summary_df.to_csv(summary_file, index=False)
    print(f"统计摘要已保存至: {summary_file}")

    return summary_df


# 示例使用


def main_rinex():
    # 配置参数
    rinex_file = "/mnt/f/PNT/实验/NMS-PNT/03_1128_Ground_Youyi/GNSS/ublox_2024_10_28_15_27.obs"  # 替换为你的RINEX文件
    dataset_dir = "/mnt/f/PNT/实验/NMS-PNT/06_1128_Ground_Youyi"
    output_csv = os.path.join(dataset_dir, "GNSS/rover_obs.csv")
    error_csv = os.path.join(dataset_dir, "evo_ape.csv")

    if not os.path.exists(output_csv):
        try:
            # 执行分析
            print("开始分析 RINEX 文件...")
            df_results = analyze_multi_gnss(rinex_file)

            # 保存结果并生成摘要
            summary = save_and_summarize(df_results, output_csv)

            # 打印统计摘要
            print("\n统计摘要:")
            print(summary.transpose().to_string(header=False))

        except Exception as e:
            print(f"处理出错: {str(e)}")

    df_results = pd.read_csv(output_csv)

    nms_plotter = NMSPlotter()
    fig, axs = plt.subplots(3, 1, figsize=(5, 7), sharex=True)
    nms_plotter.plot_rinex(df_rinex=df_results, axs=axs)
    df_err = pd.read_csv(error_csv)

    color = plt.get_cmap('Set1').colors
    # axs[2].plot([], [])
    axs[2].plot(df_err['timestamp'], df_err['RTKLIB_ape'],
                'o-', label='RTKLIB', color=color[0], linewidth=1.5, ms=2.5)
    axs[2].plot(df_err['timestamp'], df_err['VINS(Global-Fusion)_ape'],
                'o-', label='VINS(Global-Fusion)', color=color[1], linewidth=1.5, ms=2.5)
    axs[2].plot(df_err['timestamp'], df_err['IC-GVINS_ape'],
                'o-', label='IC-GVINS', color=color[2], linewidth=1.5, ms=2.5)
    axs[2].set_ylabel("ATE/m")
    # axs[2].set_xlim([374955, 375260])
    axs[0].set_ylim([10.5, 45])
    axs[1].set_ylim([0, 30])
    axs[2].set_xlim([372875, 373360])
    axs[2].legend(loc="lower left", framealpha=0.0)

    plt.subplots_adjust(
        top=0.966,
        bottom=0.091,
        left=0.134,
        right=0.952,
        hspace=0.162,
        wspace=0.2
    )
    plt.show()


def main_timestamp():
    dataset_dir = "/mnt/f/PNT/实验/NMS-PNT/01_1128_Ground_Youyi"
    df = pd.read_csv(os.path.join(dataset_dir, "adis16470.csv"))

    nms_plotter = NMSPlotter()
    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    df["field.header.stamp"] = df["field.header.stamp"] * 1e-9
    data = np.diff(
        df[["field.accel_gps_time", "field.header.stamp"]].values, axis=0)
    nms_plotter.plot_timestamp(axs, df, "ADIS16470")


if __name__ == "__main__":
    # main_timestamp()
    main_rinex()
