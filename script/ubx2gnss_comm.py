# from msg_converters import rtcm_to_meas, rtcm_to_ephem
import rospy
# from gnss_comm.msg import GnssMeasMsg, GnssEphemMsg
from pyrtcm import RTCMReader
from pyubx2 import UBXReader, UBX_PROTOCOL


def parse_ubx_file(ubx_file):
    with open(ubx_file, 'rb') as stream:
        ubr = UBXReader(stream)
        for raw, msg in ubr:
            if msg.identity == "RXM-SFRBX":
                print(f"SV: {msg.svId}, DWRD: {msg.dwrd}")
            if msg.identity == "NAV-GLOEPH" or msg.identity == "NAV-EPH":
                print(msg)
            if msg.identity == "NAV-IONO":
                print("Alpha:", msg.alpha)
                print("Beta:", msg.beta)


def parse_rtcm_file(file_path, meas_pub, ephem_pub):
    with open(file_path, 'rb') as f:
        reader = RTCMReader(f)
        for _, msg in reader:
            if msg.identity in ["1004", "1012", "1124"]:  # 双频观测
                print(msg)
            elif msg.identity in ["1019", "1020", "1045", "1042"]:  # 星历
                print(msg)


if __name__ == "__main__":
    # rtcm_file = "/mnt/f/PNT/实验/1128/ublox/ublox_2024_10_28_15_27.ubx"
    # # parse_rtcm_file(ubx_file, None, None)
    # ubx_file = "/mnt/f/PNT/实验/1128/ublox/ublox_2024_10_28_16_02.ubx"
    # parse_ubx_file(ubx_file)

    print("---end---")
