# Mirobot-ROS-control-interface
ROS service encapsulation instruction interface of mirobot robot.
该ROS包提供了Mirobot机械臂一些基本指令的ROS服务封装，以服务的形式发布机械臂控制指令
ROS版本：kinetic 1.12.17
作者：Thor zhou
zhoudongxv@yeah.net

# 使用方法
## 1.编译
```bash
$ catkin_make
```
## 2.设置环境变量
```bash
$ echo "source ~/Mirobot-ROS-control-interface/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```
## 3.运行roscore
```bash
$ roscore
```
## 4.连接机械臂USB线并查看机械臂在Ubuntu系统中对应的设备
```bash
$ ls /dev/ttyUSB0
$ sudo chmod +777 /dev/ttyUSB0
```
## 5.启动mirobot的ROS服务节点
```bash
$ rosrun mirobot MirobotServer /dev/ttyUSB0
```
其中参数字符串“/dev/ttyUSB0”为机械臂对应的设备
连接机械臂成功显示：
```bash
zdx@zdx-VirtualBox:~$ rosrun mirobot MirobotServer /dev/ttyUSB0
[ INFO] [1645021093.508624958]: Mirobot has been connected successfully!
[ INFO] [1645021093.515160083]: Mirobot service and topic running...
```
## 6.查看支持的服务
使用rosservice list查看支持的服务：
### /MirobotServer/GetPoseCmd  
获取机械臂当前位置与姿态指令，没有参数，发布后返回当前位姿值与6个关节角度值
### /MirobotServer/SetHomeCmd
机械臂复位，没有参数，发布后机械臂执行复位，返回执行结果，1成功，-1为失败。
机械臂初始上电打开后，必须执行复位指令，否则后续的运动指令将不执行，机械臂处于锁定状态。
### /MirobotServer/SetCartAbsoluteCmd
机械臂笛卡尔坐标方式移动服务指令，该指令发布的位姿为绝对位姿，参数为末端位置与姿态（RPY角），速度，发布后返回执行结果，如果成功返回1，失败返回-1
该指令无插补，为快速到达指令
例如：
```bash
$ rosservice call /MirobotServer/SetCartAbsoluteCmd "{x: 200.0, y: 10.0, z: 50.0, a: 0.0, b: 0.0, c: 0.0, speed: 2000}" 
```
运动到位置（200,10,50），姿态（0,0,0）处，速度2000
### /MirobotServer/SetCartRelativeCmd
功能与SetCartAbsoluteCmd服务类似，区别在于发布的是相对位置值
例如：
```bash
$ rosservice call /MirobotServer/SetCartAbsoluteCmd "{x: 10.0, y: 0.0, z: 0.0, a: 0.0, b: 0.0, c: 0.0, speed: 2000}" 
```
机械臂向X方向运动10，速度2000
### /MirobotServer/SetJointAbsoluteCmd
机械臂关节运动指令，参数为关节运动值，该指令为绝对位置值，返回执行结果，1为成功，-1为失败
例如：
```bash
$ rosservice call /MirobotServer/SetJointAbsoluteCmd "{jointAngle_1: 90.0, jointAngle_2: 0.0, jointAngle_3: 0.0, jointAngle_4: 0.0, jointAngle_5: 0.0,
  jointAngle_6: 0.0, speed: 2000}" 
```
机械臂关节1运动到绝对角度值90度处，速度2000
### /MirobotServer/SetJointRelativeCmd
功能与SetJointAbsoluteCmd服务类似，区别在于发布的是相对运动角度值
例如：
```bash
$ rosservice call /MirobotServer/SetJointRelativeCmd "{jointAngle_1: 10.0, jointAngle_2: 0.0, jointAngle_3: 0.0, jointAngle_4: 0.0, jointAngle_5: 0.0,
  jointAngle_6: 0.0, speed: 2000}" 
```
机械臂关节1正向运动10度，速度为2000
### /MirobotServer/SetGcodeCmd
该服务可以发布Mirobot机械臂支持的任意G代码指令，参数为要发布的G代码指令，返回值为执行结果，1为成功，-1为失败
例如：
```bash
$ rosservice call /MirobotServer/SetGcodeCmd "gcode: '\$h'"
```
该指令为机械臂复位运动指令，注意指令中的$符号前必须添加\,否则会被转义
```bash
$ rosservice call /MirobotServer/SetGcodeCmd "gcode: 'M21G91G0X10'"
```
该指令为机械臂支持的角度运动指令
