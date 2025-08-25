# A Ground Robot Dataset for Multi-Sensor Navigation in Diverse Environments
> Ban Li, Jianghui Geng, Pai Wang, Yuanxin Wu, Hui Cheng, and Hang Shi

Reliable positioning remains a challenge in autonomous ground robotics, particularly in complex and dynamic real-world environments. The fusion of drift-free global positioning and precise local positioning is essential to ensure continuous and accurate localization in mobile ground robots. However, a benchmark dataset encompassing challenging scenarios for both global and local positioning is still lacking, which limits further research and comprehensive evaluation of fusion-based Simultaneous Localization and Mapping (SLAM) methods for autonomous ground robots. To fill this gap, we introduce a ground robot dataset for multi-sensor navigation in diverse environments. All sensors are well calibrated, and Global Navigation Satellite System (GNSS), Inertial Measurement Unit (IMU), camera, and Light Detection and Ranging (LiDAR) measurements are hardware-synchronized. Furthermore, several auxiliary sensors are also included in our system, which are often overlooked in existing datasets but may be vital in certain applications. We perform data acquisition in a variety of challenging environments, both outdoors and indoors. In outdoor scenarios, ground truth is provided by a high-level integrated navigation system, while in indoor environments, it is obtained using a motion capture system. We evaluate the positioning performance of several baseline algorithms on our dataset, and the results show that current methods need further improvement in specific challenging scenarios. To advance relevant research, we make the dataset and associated tools publicly available.
## Dataset Overview
### Platform
The data acquisition platform consists of the same two-tier sensor-laden rack mounted on two unmanned ground robots for outdoor and indoor data collection.

<table>
<tr>
    <td align="center">
    <img src="./fig/outdoor.jpg" alt="Outdoor" width="240"><br>
    <span>(a) Outdoor platform</span>
    </td>
    <td align="center">
    <img src="./fig/indoor.jpg" alt="Indoor" width="240"><br>
    <span>(b) Indoor platform</span>
    </td>
</tr>
</table>

A schematic illustration of the relative positions and orientations of the sensors in our system. The upper tier houses the GNSS antenna, LiDAR, and ultrasound receiver, while the lower tier accommodates the stereo camera. The IMU, magnetometer, and barometer are embedded within the onboard computer. Additionally, six UWB tags are positioned around the rack to form a receiver array. The exact extrinsic parameters between sensors are provided in the configuration file of each data sequence.

<img src="./fig/sensor setup.jpg" alt="示意图" width="540">

### Challenging Scenarios
The dataset was collected at three scenarios: Wangan Playground, Youyi Square, and Yunzhi Experimental Field.

<table>
<tr>
    <td align="center">
    <img src="./fig/traj1.jpg" alt="Wangan Playground" width="180"><br>
    <span>(a) Wangan Playground</span>
    </td>
    <td align="center">
    <img src="./fig/traj2.jpg" alt="Youyi Square" width="180"><br>
    <span>(b) Youyi Square</span>
    </td>
    <td align="center">
    <img src="./fig/traj3.jpg" alt="Yunzhi Experimental Field" width="180"><br>
    <span>(c) Yunzhi Experimental Field</span>
    </td>
</tr>
</table>


Diverse challenging conditions included in our datasets: (a) Dynamic pedestrians; (b) Dynamic vehicles; (c) Lighting change; (d) Low illumination; (e) Satellite signal obstruction; (f) Satellite signal blockage; (g) UWB signal obstruction; (h) Outdoor-indoor transition.

<img src="./fig/scenarios.jpg" alt="示意图" width="540">

### Data Sequences
| Sequence       | Distance (m) | Duration (s) | Feature                                                                                                                                   |
|----------------|--------------|--------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| open-sky_01-02 | 849.26       | 734          | Dynamic objects: pedestrians intentionally crossed in front of or continuously followed the camera in an open-sky environment.            |
| open-sky_03-04 | 849.90       | 743          | Low illumination: sequences collected in dark nighttime scenes with scarce light sources.                                                  |
| campus_01-02   | 4371.96      | 4390         | Long-term and large-scale: significant lighting changes occur during the vehicle's long-term operation of over 30 minutes.                 |
| campus_03-05   | 2396.95      | 1872         | Satellite signal obstruction: the ground robot repeatedly passed between trees and buildings.                                               |
| campus_06-08   | 2438.04      | 1827         | Satellite signal blockage: the ground robot passed through a building twice, with each blockage lasting approximately 20 seconds.          |
| campus_09      | 739.19       | 582          | Composite: includes lighting changes, dynamic pedestrians and vehicles, satellite signal obstruction, and satellite signal blockage.        |
| indoor_01-03   | 303.42       | 759          | UWB signal obstruction: UWB signal propagation is disturbed by artificial obstacles and pedestrians indoors.                               |
| transition_01-03 | 410.10     | 1475         | Outdoor-indoor transition: for each sequence, the ground robot enters the indoor area from the outside.                                    |

## Updates
- Aug. 25, 2025, upload dataset: [MSN-DE at zendo](https://zenodo.org/records/16939391?preview=1&token=eyJhbGciOiJIUzUxMiJ9.eyJpZCI6ImY2YmQxMWFhLWIzYzQtNDgxNS05NmVhLTEwOGRkNmZmYWZkNyIsImRhdGEiOnt9LCJyYW5kb20iOiI4NGVjNGYzMWExMjFlZWE1YjM5NWUxNjdhMDE4OTk4NSJ9.RST_QdY9-Rn9qpJ-VrhbHMzDFEgvVBU5bENrOsB98gMyjrO_Ly4vyyeSa_sVOdnFNjKuQBF87eNbsysNrAoKDw)

## Supports
```
cd msn_ws
catkin_make
source devel/setup.bash
rosrun msn_example sub_all
```

## Citation
If you think our work useful for your research, please cite:
```bib
@misc{msn-de,
      title={A Ground Robot Dataset for Multi-Sensor Navigation in Diverse Environments}, 
      author={Ban Li, Jianghui Geng, Pai Wang, Yuanxin Wu, Hui Cheng, and Hang Shi},
      year={2025},
}
```
