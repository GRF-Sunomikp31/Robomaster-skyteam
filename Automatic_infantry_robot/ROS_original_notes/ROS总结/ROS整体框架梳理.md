# ROS整体框架梳理

参考资料：

- https://blog.csdn.net/zhao_ke_xue
- https://blog.csdn.net/crp997576280

**框架：**

![12](/home/nuc/Desktop/SLAM/slam_car/IMG/12.png)

**开源功能包：**

- explore_lite:自动探索环境，直到找不到边界为止，其最终调用的还是move_base;
- gmapping:开源建图包
- rplidar:思岚雷达ROS包
- move_base:自主导航框架
- acml:定位
- navigation：这个一般也都是用move_base

**ROS小车启动功能包（底盘功能包）**

一般这个功能包命名都是什么什么bringup；

功能：

- 订阅cmd_vel 话题
- 与STM32串口通信
- ROS小车里程计odom计算
- 发布里程计odom和tf

![8](/home/nuc/Desktop/SLAM/slam_car/IMG/8.png)

**ROS小车启动的launch文件**

功能：

- 启动机器人启动功能包ROS节点
- 加载机器人模型参数文件
- 发布机器人关节状态
- 发布机器人tf变换，由机器人模型文件维护的部分
- 启动激光雷达等传感器驱动功能节点



# 其他思考

## 其他

1、我们在分析一个功能包功能时：应该分析  订阅的话题和发布的话题

2、gampping算法不需要里程计：gmapping是需要里程计信息的！而且非常依赖里程计数据的准备性

如果说，如果gmapping不需要里程计信息，那么odom的功能是什么？tf变换间接需要

**cartographer**是不需要依赖里程计的。

![10](/home/nuc/Desktop/SLAM/slam_car/IMG/10.png)



odom 里程计   scan  激光数据   激光雷达的数据（/scan）、TF变换树和初始位姿估计（/Initialpose）

激光雷达的启动包发布topic ： /scan

2D激光雷达建图的算法一般有Gmapping、Hector、Cartograph。

3D激光雷达的建图算法有谷歌的Cartographer算法和LeGO-LOAM

## 建图

gmapping算法：

![11](/home/nuc/Desktop/SLAM/slam_car/IMG/11.png)

## **导航**

导航这块感觉有点乱，很多经典框架、包；move_base、explore_lite、navigation，还有全局规划期、局部规划器。。。

move_base最后的输出的 /cmd_vel  这个话题的数据

## **定位**

这个没什么好说的，就acml，基于激光导航的小车中定位数据来源有两个地方轮式里程计和AMCL包

acml：

## tf

tf数据的发布，是以广播的形式发布的；

## 关节信息

可以不写机器人3D模型文件，自己添加：

安装位置手动发布激光雷达(laser)和底盘（base_link）之间的变换信息（所谓变换就是两个坐标系之间的额相对旋转和平移）

具体就是 直接调用`tf`的`static_transform_publisher` 节点，并给一些参数；

`static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms`

x y z分别代表着相应轴的平移，单位是米 。

yaw pitch roll 分别代表着绕三个轴的转动，单位是 弧度  ，yaw pitch roll 分别对应着 Z，Y，X轴的旋转，也就是把我们总说的XYZ的反过来，只要记住顺序还是不容易弄错的。

frame_id为坐标系变换中的父坐标系， child_frame_id为坐标系变换中的子坐标系。

最后一个参数为发布频率，单位为 毫秒。通常取100。  一毫秒为一秒的千分之一，100毫秒即为0.1秒，也就是10Hz。

虽然参数有七个，但是有两种发布形式：https://blog.csdn.net/qq_29797957/article/details/103829918

- 俯仰角+坐标位置
- 四元数+坐标位置

这里如何通过urdf描述关节信息，如果自己创建这个模型

节点坐标位置问题：如何选取？

## RVIZ

Rviz中如何实时显示SLAM算法的建图信息?

- 运行roscore，打开rviz
- 左下角add添加map
- map中topic选择/map

然后并没有建图，是因为我这边只运行了上位机代码

Rviz中如何实现导航：

- 在rviz界面点击“2D Nav Goal”按钮，这个按钮用于设置导航的目标点；
- 鼠标左键点击目标点不要松开，选择方向后再松开；

按道理是这样，但是rviz怎么进行信息交互的？好像就是可以直接导航

参考 ：https://www.ncnynl.com/archives/201708/1884.html

# 问题

1、开一个分享专门分享激光雷达，激光雷达直接输出的数据类型，以及利用ROS包输出的数据类型

激光雷达是有一个ip地址的，这个ip地址什么作用？？、

激光雷达的坐标系关系

2、关于坐标关系的tf在哪配置？description这个包



现在看来在用键盘控制小车时，纯gmapping建图是不需要acml定位的，那在rviz中的位置信息从哪来的？

acml和导航包是一起的？**键盘控制小车是不需要acml的**

还有gmapping包的具体功能，因为感觉由激光点生成地图是一件很简单的事情，gmapping主要在匹配特征点？

launch文件中的param设置参数，这些参数在功能包中都有初始化定义吗？



利用导航功能包的话，rviz中实现导航有什么怎么发数据的，发送什么数据呢？



cmd_vel 和遥控器控制原理是否相同？



# 使用总结

## 代码总结

**1、键盘控制建图**

-  ROS启动节点：
- gmapping节点：启动独立
- 激光雷达启动节点：依赖激光雷达端口号
- 键盘控制节点：启动独立

**2、导航建图**

-  ROS启动节点
- gmapping节点
- 激光雷达启动节点
- acml节点
- move_base节点

这里我觉得还差维护tf树的文件，机器人模型坐标变换的文件；

## 其他总结

### 激光雷达

- 思岚激光雷达ROS包，A3和A2/A1的区别就是，A3波特率在256000，A2和A1波特率在115200；并且，A3有  <param name="scan_mode"           type="string" value="Sensitivity"/>这项参数

- 一般在激光雷达启动launch文件中指定激光雷达的端口号，用来识别相应的端口；
- 按道理利用launch文件启动的rplidarNode，应该是和src中的Node节点对应的。

### ROS基础知识总结

- param：提供一个接口可以修改节点中的参数，一般这个参数节点文件中有初定义

- 复制一个功能包，要在cmakelist和package中添加依赖，在功能包下的cmakelist中的build添加以下两行

`add_executable(mick_bringup_test2 src/mick_bringup_test2.cpp)`
`target_link_libraries(mick_bringup_test2`
	`${catkin_LIBRARIES}`
	`${OpenCV_LIBRARIES}`
`)`

- 当spinOnce函数被调用时，spinOnce就会调用回调函数队列中第一个callback函数，此时callback函数才被执行，然后等到下次spinOnce函数又被调用时，回调函数队列中第二个callback函数就会被调用，以此类推。


  所以，这会有一个问题。因为回调函数队列的长度是有限的，如果发布器发送数据的速度太快，spinOnce函数调用的频率太少，就会导致队列溢出，一些callback函数就会被挤掉，导致没被执行到。

  而对于spin函数，一旦进入spin函数，它就不会返回了，相当于它在自己的函数里面死循环了。只要回调函数队列里面有callback函数在，它就会马上去执行callback函数。如果没有的话，它就会阻塞，不会占用CPU。

- `ser.available()<33){ROS_INFO("wait");}`   保证读取大于33byte数据,即一个完整的数据包
- `ros::ok()`返回false，代表可能发生了以下事件 1.SIGINT被触发(Ctrl-C)调用了ros::shutdown() 2.被另一同名节点踢出 ROS 网络 3.ros::shutdown()被程序的另一部分调用 4.节点中的所有ros::NodeHandles 都已经被销毁
- rosrun运行节点的时候加参数：    `rosrun tf static_transform_publisher _x:=0.0 _y:=0 _z:=0.5 _qx:=-3.1415926 _qy:=0 _qz:=0 _qw:=0 _frame_id:=base_link _child_frame_id:=laser _period:=50`  
- launch文件是不需要编译的
- `ll /dev | grep ttyUSB`  查看串口连接状态；我现在nuc绑定的串口mick串口只识别ch340这个了。
- rosed:编辑ROS中的文件

# 小车调试

**1、先通过键盘控制调试小车基本运动**
现在可以直接通过teleop_keyboard发布cmd_vel数据，mick_bringup接受并做分解通过串口发送给底盘，

现在的问题是

- 通信协议：只保留一个通信协议，他本身代码中 转速控制这个代码没有使用我去掉；

- 超声波：这块要去掉，但是我没有搞懂这个的原理，他的数据通过什么发布过去呢？

- ROS小车底盘为什么要清楚里程计？也就是说STM32给单片机传的是什么数据？

- 这里还有一个问题，底盘节点接收到cmd_vel 的指令时，通信协议只发一次，小车应该的数据应该走多长呢？自己给这个数据，自己给这个时间

- 从整体程序设计的角度，为什么要一直清除里程计数据；

- ros如何实现一个底盘节点同时收发数据的？

  **代码分析：**

- `while(!init_OK)`  程序检测下位机是否有数据发出，否则就一直清0
- `analy_uart_recive_data( std_msgs::String serial_data)`   这个函数虽然返回值是bool，检测通信协议，但是其内部分解了通信协议；其需要引出的变量都定义为了全局变量；

这里程序会一直卡在这里，除非串口那边有发数据，否则一直都是清零的，问题来了，程序卡在这里怎么会接受数据呢

还有个问题，他这个代码电机数据和IMU数据是分开的

已解决这个问题：代码  mick_bringup_test0

**2、通过键盘控制调试小车建图**

这个就要牵扯到小车的车型大小，雷达位置描述了；

现在问题：

- tf节点变换的来源以及是否必须存在？laser坐标系和lase_link坐标系的定义
- 底盘节点中，如果接受数据就会一直卡在那里从而不能去发送数据，所以底盘发送数据的频率应该为多大？
- tf坐标变换关系base_link的参考点在哪？
- 整体框架结构：键盘控制节点-底盘节点-gmapping节点-雷达节点-tf坐标系变换节点
- 现在需要用到里程计了就要话说回来了？里程计需不需要一直清零?
- 问题是gmapping输入的两个tf变换的关系是谁输入的？  `static_transform_publisher`发布`/tf [tf2_msgs/TFMessage]`话题；

https://www.freesion.com/article/64701318254/ 机器人描述语言同时发布这两个tf，但是上面这个只发布了一个；为什么需要两个tf  参考：https://blog.csdn.net/jcsm__/article/details/108622554

`robot_state_publisher`  节点是发布`tf_static`   但是这个节点好像只能接受关节信息   

这个数据按道理应该是底盘节点发送的；

暂时解决：代码 见代码备份

**3、调试小车建图导航**

现在问题：

- 这边就要涉及map_server，acml，move_base了；还有就是rviz中如何实现导航；move_basez中一些参数要修改；

- map_server前提必须知道地图.yaml文件，这个需要有上一步生成；









**IMU问题**：https://blog.csdn.net/zhu751191958/article/details/79322364

里程计又包含2 个方面的信息：

一、是位姿（位置和转角），即（x,y,θ）

二、是速度（前进速度和转向速度）。

问题又来了，对于那个人的程序源码，既然更新odom即需要速度又需要IMU，那为啥还是用了两条通信协议？

里程计计算可以不用IMU吗的？可以不用IMU融合得到里程计，但是很多还是选择使用IMU融合更加准确；

**实际测量**

- STM32传给PC速度的频率是20ms一次

- 坐标关系x=0.1m，y=0.2m，z=0.3

原先参数：`args="0.0 0 0.5 -3.1415926 0 0 base_link laser 50"` 

修改参数：`args="0.1 0.2 0.3 -3.1415926 0 0 base_link laser 50"` 

**看慕课：机器人操作系统mooc**

 <node pkg="tf" type="static_transform_publisher" name="link1_broadcaster" args="0.0 0 0.5 -3.1415926 0 0 base_link laser 50" />

局部地图和全局地图的问题。