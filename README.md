### IK FOR MODEL H 

## 高擎电机零位
1. 电机主轴方向为旋转正方向
2. 电机旋转一圈，编码器单位为100000
3. 机械零位：只有手肘弯曲90度，其余均为伸直状态为零位

## 运动学动态库编译
> 参考：git@github.com:rocketman123456/ros2_ws.git

1. 生成.so库：

```shell
mkdir build
cd build
cmake ..
make
```
2. 运行`ik_test.py`

```shell
python ik_test.py
```
