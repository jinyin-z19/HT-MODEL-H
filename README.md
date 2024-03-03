## HighTorque MODEL H & THMOS 

### 代码部署
> 基于高擎Model H双足机器人

1. 安装依赖库

```shell
sudo apt-get install libeigen3-dev
pip install numpy
pip install control 
```

2. 编译运动学和电机控制动态连接库

* 运动学库(xxx指代代码包所在路径)
> 参考：git@github.com:rocketman123456/ros2_ws.git
```shell
cd xxx/HT-MODEL-H/walking_packet/MODEL_H_IK_BUILDER/build
cmake ..
make
```

* 电机库(xxx指代代码包所在路径)
> 参考：https://gitee.com/guangzhou-smart-equation/communication-board-sdk/tree/master
```shell
cd xxx/HT-MODEL-H/walking_packet/MODEL_H_MOTOR_BUILDER/build
cmake ..
make
```
### 代码运行

* 运动学逆解验证调试
```shell
cd xxx/HT-MODEL-H/
sudo -E python ik_demo.py
```
* 行走演示
```shell
cd xxx/HT-MODEL-H/
sudo -E python walking_demo.py
```
> 不加sudo无法打开SPI，不加-E无法找到python包

### 其他内容

* 高擎电机零位与编号
![](MODEL.png)
1. 电机主轴方向为旋转正方向
2. 电机旋转一圈，编码器单位为100000
3. 机械零位：只有手肘弯曲90度，其余均为伸直状态为零位

* 步态参数调试
修改param.txt

* 关节零位修改
walking_demo.py中的 zero_list

* 关节转向修改
walking_demo.py中的 motor_way

