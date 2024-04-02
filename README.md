# RobotIC24赛季场地定位方案

## 当前进度

### 嵌软

~~- MPU6050与布瑞特编码轮已初步调通（23/10/31）~~

~~- keil工程文件待上传（23/10/31）~~

- 已上传，在407上使用的版本，输出为绝对坐标。（23/10/31）
- - CAN FILTER采用IDMASK 修改时需要注意
  - ~~输出坐标相较于车体坐标多偏移了45度，下次改~~ 改完了 （23/12/11）
  - 后面考虑加上卡尔曼 -> 感觉没必要，运行速度会下降，而且准不了多少。
  - 启动进入自校准之后大概需要1s左右开始输出稳定读数。
  - 这个是带了要求输出的版本
  - 等磁编码中





## 一些其他的参数

+ 轮子的直径是75 mm // 一圈 

+ 编码器一圈16384

+ 所以每编码器格 = 0,014373

## 启动场地定位可视化
- 修改Cmakelist: add_executable(talker src/SerialPath.cpp src/Serial.cpp)
- 打开一个新的终端，导航到 `ros2_path`，
```
cd  ~/ros2_path
colcon build --packages-select show_path
. install/setup.bash
ros2 run show_path talker
ros2 launch show_path show_path.launch.py
```
  
## 启动底盘闭环控制Demo
- 修改Cmakelist: add_executable(talker src/SerialControl.cpp src/Serial.cpp)
- 打开一个新的终端，导航到 `ros2_path`
  ```
  cd  ~/ros2_path
  chmod +x launch.sh
  ./launch.sh
  ```
