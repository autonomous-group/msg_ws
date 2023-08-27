# simulation_msgs
This package is used for communication between simlation platform and autonomous system.  
Since we need build trucksim simulation platform from scratch, this package came into being.  

## File structure  
├── CMakeLists.txt  
├── msg  
│   └── simulation.msg  
├── package.xml  
└── readme.md  

## msg defenition  
### simulation.msg  
string descriptor  
time[] stamps  
float64[] control  
float64[] report  

## About  
考虑到需要应用的车辆平台众多，不同车辆平台需要的控制量和能够提供的反馈量也不尽相同（如以发动机为动力和以电机为动力的车辆），尽管最初只应用于trucksim平台单车型的仿真，但仍需考虑日后基于不同车辆可能需要频繁更换控制量/接口，故这里控制量和反馈量皆以容器形式存储，每个数据的物理意义以约定定义，针对每个平台的约定皆需在此文档中翔实记录。  

## Common agreements  
- 约定中未作描述的量保留，无需任何操作
- `simulation.msg/control`中存储控制量，按照约定按顺序存储  
- `simulation.msg/report`中存储仿真平台反馈量，按照约定按顺序存储  
- `simulation.msg/stamps`中依次存储消息到达各环节的时间戳/`ros::Time`  
- 约定中冗余的量、未定义的量、无需赋值的量可赋予任意值，但不可跳过，因为所有约定依赖于变量在容器中的位置  
- 约定中为整型的量请使用强制类型转换，或：(int)std::round(variable)  
- 方向盘转角采用角度制，且左转为正，右转为负  
- 档位：N、D、R、P 依次为：0、1、2、3  
- 油门量、制动量以百分比形式给出，最小为0，最大为1
- 车速向前行驶（D挡）为正，向后行驶（R挡）为负
- 其他单位参考国际单位制


## Agreements on trucksim/20220918  
仿真平台收到消息后，保存此消息或此消息的副本，仿真平台执行消息中的控制量后，为收到的消息/副本中添加约定需要添加的时间戳/stamp以及反馈量/report，再将消息发回。  

### topics  
- 仿真平台接收控制量的话题：`/simulation_control`  
- 仿真平台发送反馈量的话题：`/simulation_report`  

### `simulation.msg/stamps`中存储的量按顺序依次如下：  
- 无人驾驶系统消息发出时的时间戳  
- 仿真平台接收到消息时的时间戳  
- 仿真平台反馈消息发出时的时间戳  
- 无人驾驶系统接收到仿真平台反馈消息的时间戳  

### `simulation.msg/control`中存储的控制量按顺序依次如下：  
- 方向盘转角  
- 档位  
- 油门量  
- 制动量  

### `simulation.msg/report`中存储的控制量按顺序依次如下：  
- 方向盘转角  
- 档位  
- 油门量  
- 制动量  
- 车速  
- x/暂定为仿真平台的局部坐标
- y/暂定为仿真平台的局部坐标
