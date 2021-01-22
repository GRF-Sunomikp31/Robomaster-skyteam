
# 2021苍穹战队视觉组寒假学习计划--环境配置

## 前言

本文为环境配置文章，寒假学习使用平台为Python++Opencv+Pytorch，推荐使用软件为Anaconda+Pytorch+Opencv+Pycharm。

开发环境：anaconda：5.3.1（版本不同基本无差异）

​					Pytorch：1.7

​					Opencv：4.0.1

​					Cuda（选）10.2

​					Pycharm community

## Anaconda

### 安装

首先安装anaconda，python包管理工具，可以方便的下载管理第三方软件库，同时自带python语言。

官网下载链接：https://www.anaconda.com/products/individual#Downloads

然后选择合适的版本下载：

![QQ截图20210122100257](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122100257.png)

后面下载好了直接安装，下面这个选项记得添加一下，其他的暴力安装即可，另外安装路径C盘非必须。

![QQ截图20210122100724](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122100724.png)

### 测试

CMD+R进入windows命令行交互工具，输入cmd，进入后，输入conda list 显示很多用conda安装的包即可表示anaconda安装成功。

![QQ截图20210122101200](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122101200.png)

或者 使用 conda --version查看conda的版本

![QQ截图20210122101619](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122101619.png)

## Cuda(选)

Cuda是NVIDIA推出，可以用加速计算的并行计算语言库；这里没有NIVIDA显卡可以直接忽略这步，不安装。

### 安装

Cuda最新版本的是11.2，但是现在pytorch还不支持这个版本，因此我们选择旧版本，如10.2

下载地址：https://developer.nvidia.com/cuda-10.2-download-archive

具体选择如下：

![](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122103408.png)

下载好后直接安装即可。

### 测试

安装好后，在安装目录下能找到 nvcc 这个程序（Cuda软件的编辑器）

![QQ截图20210122103741](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122103741.png)

进入到cmd下，输出nvcc -V输出内容如下，即证明安装成功

![QQ截图20210122103939](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122103939.png)

## Pytorch

### 安装

进入pytorch官网：https://pytorch.org/

选择配置，复制命令行

![QQ截图20210122104732](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122104732.png)

**管理员身份打开cmd**

![QQ截图20210122104818](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122104818.png)

复制命令直接安装

![QQ截图20210122104931](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122104931.png)

## Pychram

### 安装

进入官网下载页面：https://www.jetbrains.com/pycharm/download/#section=windows

选择free社区版下载

![QQ截图20210122105242](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122105242.png)

然后直接安装即可，这里要注意的时**pycharm一定要装到C盘**（固态），装到机械硬盘上会非常卡！

### 测试

打开pycharm

file-new project  然后按照下面提示将conda安装的python.exe路径导入即可

![QQ截图20210122111721](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122111721.png)

测试代码

```python
import torch
print(torch.__version__)
print('gpu:',torch.cuda.is_available())
```

正常运行输出即可：

![QQ截图20210122113102](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122113102.png)

## Opencv

### 安装

选择file-setting-Project：pythonProject -Python：Interpreter-conda的那个python环境-再点击+号搜索opencv安装即可

![QQ截图20210122105736](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122105736.png)

### 测试

测试代码，功能：打开摄像头

```python
# -*- coding=GBK -*-
import cv2 as cv
 
 
#打开摄像头获取图片
def video_demo():
    capture = cv.VideoCapture(0)#打开摄像头，0代表的是设备id，如果有多个摄像头，可以设置其他数值
    while True:
        ret, frame = capture.read() #读取摄像头,它能返回两个参数，第一个参数是bool型的ret，其值为True或False，代表有没有读到图片；第二个参数是frame，是当前截取一帧的图片
        frame = cv.flip(frame, 1)#翻转 0:上下颠倒 大于0水平颠倒   小于180旋转
        cv.imshow("video", frame)
        if cv.waitKey(10) & 0xFF == ord('q'): #键盘输入q退出窗口，不按q点击关闭会一直关不掉 也可以设置成其他键。
            break
 
 
video_demo()
cv.destroyAllWindows()
```

运行程序，打开摄像头，即证明安装成功;

![QQ截图20210122110449](C:\Users\lenovo\Desktop\视觉组学习计划\环境配置\picture\QQ截图20210122110449.png)

## 其他工具

### Github：

因为后续的学习内容都会发在github上，所以我们建议你有一个github账号；

网站链接：https://github.com/

## CSDN：

CSDN：我们推荐使用github记录笔记及作业内容，但是它对初学者可能有点难度，当然你也可以选择方便入手的CSDN。

### Typora：

typora是一款优秀的笔记记录工具，并且默认Markdown的语法，上面编辑好的笔记也可以直接传到CSDN或者github上，我们强烈推荐你使用这款工具。
