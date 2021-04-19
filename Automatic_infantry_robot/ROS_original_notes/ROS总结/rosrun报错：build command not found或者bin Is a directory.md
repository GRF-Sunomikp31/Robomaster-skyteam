# rosrun报错：build: command not found或者/bin: Is a directory

## 具体报错如下：

![14](/home/nuc/Desktop/SLAM/slam_car/IMG/14.png)

## 解决方案：

这个问题是命令行敲错了，应该运行：`rosrun mick_bringup mick_bringup`，错误运行了：`rosrun mick_bringup mick_bringup.cpp`   （多加了.cpp，应该运行可执行文件的

## 为什么会敲错：

首先肯定是tab键补全的问题，但是我这个问题在于这个节点.cpp文件并没有编译生成可执行文件，导致一tab就直接定位到.cpp文件上了；

这个节点文件是我复制过来的，然后在Cmakelist中也添加了声明如下：

`add_executable(mick_bringup_test2 src/mick_bringup_test2.cpp)`
`target_link_libraries(mick_bringup_test2`
	`${catkin_LIBRARIES}`
	`${OpenCV_LIBRARIES}`
`)`

但是添加之后，使用`catkin_make` 没有编译这个节点（按道理说，应该是没有编译没有访问那个Cmakelist文件），就没有生成可执行文件；

*其实这里是可以重新下载这个功能包，编译之前先修改，在编译（因为我觉得第一次肯定编译了，后面应该没编译）；我觉得这种方法可行的，但是想找出真正的问题所在以及真正的解决方法。*

其他人之前也遇到过这样的问题：https://blog.csdn.net/dingjianfeng2014/article/details/78675639

**解决方法**：删除 build和devel目录的文件，再重新编译即可；

编译好了之前别忘记再打开一个新的终端。

## 其他问题

1、cloud_msgs/cloud_info.h: No such file or directory

参考：https://ask.csdn.net/questions/3334153

解决方法：`catkin_make -DCATKIN_WHITELIST_PACKAGES="cloud_msgs"`

