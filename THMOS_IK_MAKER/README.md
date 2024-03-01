### 对比cpp和py计算速度

1. 创建C++的源文件`thmos_leg_ik_raw.cpp`

2. 使用Eigen库需要将编译选项加入到CMakeLists.txt文件中

3. 生成.so库：

```shell
mkdir build
cd build
cmake ..
make
```
4. 运行`ik_speed_compare.py`以对比计算速度

```shell
python ik_speed_compare.py
```
