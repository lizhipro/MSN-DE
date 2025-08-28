LP mode
1.system_time刷新是否取决于anchor0的连接
ans:是，local_time不会刷新。
1）取消anchor0上电时，system_time会继续变化，anchor0对应的数据会不变，而不是归0
2）继续anchor0上电时，system_time会从0开始刷新。anchor0对应的数据会先于system_time刷新，大概2s内。
3）取消tag上电时，对应串口的open和数据发布程序会停止；再次上电，需要重新roslaunch，local_time会刷新

------------------------------------------------
2.systemtime刷新是否取决于console的连接
1） 当console上电时，system_time和local_time相同，都会刷新。anchor0不会影响

------------------------------------------------
3.多基站的数据采集是否正常（大于4个基站）
ans：ok

------------------------------------------------
4.多基站多tag的数据采集是否正常（大于2个tag）
ans:ok

------------------------------------------------
5.多tag的数据是否会存在时间乱序
ans:暂时未发现

------------------------------------------------
6.数据格式

直连tag
文件：LinktrackTagframe0.msg
role: 2
id: 2
local_time: 20868220
system_time: 20866760
utc_time: 1724231568623
voltage: 4.8979997634887695
pos_3d: [1.0, 1.0, 1.0]
eop_3d: [2.549999952316284, 2.549999952316284, 2.549999952316284]
vel_3d: [0.0, 0.0, 0.0]
dis_arr: [0.0, 0.0, 0.0, 0.0, 0.0, 1.215000033378601, 0.0, 0.0]
angle_3d: [-109.91999816894531, 92.98999786376953, 67.36000061035156]
quaternion: [-0.5979426503181458, 0.4443919062614441, -0.5736963748931885, -0.3353838622570038]
imu_gyro_3d: [-0.008234619162976742, 0.0003835010575130582, 0.016177352517843246]
imu_acc_3d: [-0.12349694222211838, -9.662090301513672, -0.05454542487859726]
---

直连console
LinktrackAnchorframe0.msg
role: 3
id: 0
local_time: 12651
system_time: 12651
voltage: 4.901000022888184
nodes: 
  - 
    role: 2
    id: 6
    pos_3d: [1.0, 1.0, 1.0]
    dis_arr: [1.3700000047683716, 0.9300000071525574, 1.2200000286102295, 2.2200000286102295, 0.0, 0.9900000095367432, 0.0, 0.0]
---


------------------------------------------------
7.连接tag获取数据的方式
1）roslaunch nlink_parser linktrack.launch
2）rostopic echo /nlink_linktrack_tagframe0

8.连接console获取数据的方式
1）roslaunch nlink_parser linktrack.launch
2）rostopic echo /nlink_linktrack_anchorframe0

3）获取方式code
linktrack.main.AddNewData-> nlink_protocol.HandleData-> advertise && publish(msg)
(只能获取4个最近的anchor距离)，原因：串口读出来的原始数据只有最近的四个距离更新了

9.数据中不包含UTC时间戳
1）修改msg文件和msg对应的格式文件LinktrackTagframe0.h
2）在从串口读取数据时，获得时间戳。
3）发布msg时，将时间戳赋予msg

注意：修改msg文件会导致md5值发生变化，rostopic时会报错。需要根据报错找到原md5值的文件位置，改为报错中新的md5值


