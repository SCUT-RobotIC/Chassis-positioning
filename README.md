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
