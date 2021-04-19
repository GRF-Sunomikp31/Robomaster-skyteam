# 学习过程记录

2021/3/25 
今天目标：调试激光雷达
参考资料  
https://blog.csdn.net/sinat_25923849/article/details/108521603 

`roslaunch rplidar_ros view_rplidar.launch` 报错如下：

![1](/home/nuc/Desktop/SLAM/slam_car/IMG/1.png)

`sudo chmod 777 /dev/ttyUSB0` 之后关机重启。

可以扫描出点云：

![2](/home/nuc/Desktop/SLAM/slam_car/IMG/2.png)

https://blog.csdn.net/sinat_25923849/article/details/108527802

编写launch文件后，用roslaunch运行提示找不到文件，这时候添加环境变量，`source ~/catkin_ws/devel/setup.bash`最好直接添加在.bashrc中；

前面那位朋友的教程只能实现ROS下调试激光雷达基本方案；后面想跟着下面的教程按照搭建小车的思路来；

参考资料：https://blog.csdn.net/zhao_ke_xue/article/details/108425922  ROS车里程计模型（坐标系以及各种变换）

2021/3/28

机器人小车整体上位机代码框架：

（代码框架图）

另一方面，如何使用开源算法框架搭建一个小车?

（小白机器人）ROS小车代码框架：

![3](/home/nuc/Desktop/SLAM/slam_car/IMG/3.png)

`catkin_create_pkg robot_bringup tf roscpp rospy std_msgs`    这是创建一个功能包空间；

关于设备串口号的问题：雷达和USB转ttl在linux下的串口号都是/dev/ttyUSB0...，默认第一个分配为0，后面继续分配；设备固定串口号的方法：https://blog.csdn.net/zhao_ke_xue/article/details/108700080

`Resource not found: robot_description`

这个要根据之前的文章自己添加一下机器人模型文件：https://blog.csdn.net/zhao_ke_xue/article/details/108396430  用urdf文件或者xacro文件。

话又说回来，为什么需要机器人模型文件？在rviz中做导航不就是一个点吗？

我觉得我一切都配置好了，但是运行`roslaunch robot_bringup robot_bringup.launch`报错如下：

![4](/home/nuc/Desktop/SLAM/slam_car/IMG/4.png)

这里应该是缺少雷达的launch的文件，但是我使用`sudo apt-get install ros-melodic-rplidar-ros`安装不行；

这里我把一开始参考另外一个博主下载的A1雷达的驱动文件中的rplidar.launch 放在该功能包launch下，报错解决，但在运行时出现新的报错：并且雷达不转，明天解决。。。

2021/3/29

![5](/home/nuc/Desktop/SLAM/slam_car/IMG/5.png)

昨天这个报错是因为雷达和usb转ttl的端口号相反导致的，现在launch文件能够正常运行。这个代码中USB转ttl设备是：ttyUSB1，激光雷达是ttyUSB0.

设备串口地址固定和开机自启现在还没有必要去设置；

下一篇博客：https://blog.csdn.net/zhao_ke_xue/article/details/110916137  测试建图导航。

`explore.launch` 这个不需要修改。

上一篇博客运行代码又问题。

`sudo apt-get purge ros-melodic-explore-lite`   ros中删除一个功能包；  roscd 进入到一个ros功能包下；

现在的问题就是：很多教程有问题，走不到最后；而且整体框架还没理清；

现在问题：整体框架；开源功能包；

调试激光雷达：

使用`roslaunch rplidar_ros view_rplidar.launch`  命令正常显示激光雷达显示参数；

但是我想直接启动激光雷达（作为小车建图中的一环），`roslaunch rplidar_ros rplidar_a3.launch`     结果报错：

`[ERROR] [1617460205.131680621]: Error, operation time out. RESULT_OPERATION_TIMEOUT!` 
`[rplidarNode-2] process has died [pid 22521, exit code 255, cmd /home/nuc/catkin_ws/devel/lib/rplidar_ros/rplidarNode __name:=rplidarNode __log:=/home/nuc/.ros/log/1224236e-9489-11eb-b8ff-44af2808ddfb/rplidarNode-2.log].`
`log file: /home/nuc/.ros/log/1224236e-9489-11eb-b8ff-44af2808ddfb/rplidarNode-2*.log`

![13](/home/nuc/Desktop/SLAM/slam_car/IMG/13.png)

解决方法：参考资料：https://blog.csdn.net/zkk9527/article/details/109177275  从这个launch文件上看 波特率256000 应该是a3的；

 `rplidar_a3.launch` 这个应该不是a1通用的，直接改波特率不行；

这里更正，激光雷达传感器并不会识别芯片，而是默认识别`/dev/ttyUSB0` 建立直接看lidar的launch文件。

现在的问题是， `rplidar_a3.launch`这个launch文件不同，之前测试的可以的 `view_rplidar.launch `  里面调用了rplidar_ros下的rplidar.launch  但是这个launch文件一直找不到；

暂且搁置这个问题，rplidar_ros中`test_rplidar.launch`  中是a1的可以用这个测试

# 笔记

ROS中常用命令：

```bash
#最常用
roscore                    #启动rosmaster
rosrun  pkg_name node_name #启动ros节点
roslaunch pkg_name launch_files_name #启动launch文件
catkin_make                #编译工作空间
rospack profile            #刷新功能包路径

#环境变量
echo $ROS_PACKAGE_PATH     #打印ros环境变量
export | grep ROS          #确认环境变量已经设置正确
source devel/setup.bash    #刷新环境变量
echo "source ~/catkin_test_ws/devel/setup.bash" >> ~/.bashrc    #刷新环境变量，永久有效
source ~/.bashrc                                                #生效上一句

#功能包
catkin_create_pkg test_package std_msgs roscpp rospy            #创建名字为test_package的功能包，添加std_msgs roscpp rospy依赖
rospack list               #查看软件包列表
rospack find package-name  #定位软件包
roscd package-name         #切换到指定功能包目录

#话题
rostopic list              #输出当前运行的topic列表
rostopic info topic-name   #查看话题信息
rostopic echo topic-name   #输出话题数据
rostopic hz topic-name     #每秒发布的消息数量
rostopic bw topic-name     #每秒发布信息所占的字节量

#工具
rviz                       #启动rviz
rqt_graph                  #可视化节点关系
rqt_plot                   #可视化话题数据
rosrun rqt_tf_tree rqt_tf_tree #查看tf树

#数据记录与播放
rosbag record -a           #录制所有topic到bag文件
rosbag play bag_files_name #播放bag文件
```

关于上下位机通信协议要注意的几点内容

- 具体通信协议格式
- 使用结构体封装通信协议
- 一个字节最大255，考虑如何用多个字节储存大点的数，以及如何存储小数格式；这里博客给的方式是用共用体的方式传输大于一个字节的数据：https://blog.csdn.net/zhao_ke_xue/article/details/105493907
- 题外话：可以发送字母吗？转移到ascii码传输吗？
- 通信协议如何传输负数，因为车轮子的运动方向是有正负之分的。

```bash
source devel/setup.bash              							#刷新环境变量，单次有效
echo "source ~/catkin_test_ws/devel/setup.bash" >> ~/.bashrc    #刷新环境变量，永久有效
source ~/.bashrc                                                #执行上面一个语句，需要执行这一句
```

`rospack profile`  创建一个功能包时，如果在调用的时候找不到这个功能包，可以用以下命令刷新功能包路径；

机器人学中几种笛卡尔坐标系：**机器人坐标系XR YR OR** 、**传感器坐标系XS YS OS** 、**世界坐标系XW YW OW**；世界坐标系固定不变，传感器和机器人坐标系都是在世界坐标系下的描述；所以最后传感器采集的数据也要转换到机器人坐标系或者世界坐标系下使用；

一般世界坐标系的选取：默认世界坐标系的X正半轴为正方向，逆时针旋转为旋转正方向，初始时刻机器人坐标系和世界坐标系重合；

移动机器人里程计模型：移动机器人的里程计就是机器人每时每刻在世界坐标系下位姿状态；这里不同小车（底盘）有不同的模型，差速两轮、普通四轮、麦轮四轮；所以要建立机器人里程计模型首先就要分析小车运动；

首先对与电机编码器，在转动一定角度后会输出一个脉冲，结合单片机时钟就可以知道电机角速度，结合轮子大小就可以计算单个轮子的线速度，线速度再积分就得到位移，四个轮子位移合成分解，得到整个车的运动里程计。





# 其他思考

1、如何实现利用ROS实现多机通信：https://blog.csdn.net/zhao_ke_xue/article/details/108314085

2、在gazebo中使用URDF搭建小车模型：https://blog.csdn.net/zhao_ke_xue/article/details/108396430

3、利用QT给ROS写一个上位机：https://blog.csdn.net/zhao_ke_xue/article/details/113790666

4、激光雷达输出数据的格式？

思岚A1 激光雷达角度分辨率≤1，输出的每一个激光点的数据都是使用极坐标的方式描述的，一个距离值以及一个对应的角度值。

5、现在IMU可以直接插在电脑上做调试吗？
6、这个小白学机器人的代码解析一个雷达的位置和机器人的位置是怎么确定的？