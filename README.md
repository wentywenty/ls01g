# LS01G 激光雷达 ROS2 驱动

这个包提供了乐动机器人 LS01G 激光雷达的 ROS2 驱动。此驱动支持实时数据采集、可视化及与其他 ROS2 节点的集成。

## 目录

- 硬件要求
- 软件依赖
- Windows 与 Linux 之间的 USB 设备共享
- 设置 udev 规则
- 创建 ROS2 工作区
- 编译
- 运行
- 功能说明
- 参数配置
- 故障排除
- 开发指南
- 许可说明

## 硬件要求

- LS01G 激光雷达
- USB 连接线（激光雷达使用 CP210x 芯片进行 USB 通信）
- 计算机（支持 Linux 或 Windows+WSL）
- 激光雷达供电（部分型号需要外部供电）

## 软件依赖

- ROS2 (Humble 或以上版本)
- CP210x USB 转串口驱动
- 以下 ROS2 包：
  - rclcpp
  - std_msgs
  - sensor_msgs
  - geometry_msgs
  - robot_state_publisher
  - joint_state_publisher
  - tf2_ros
  - rviz2
  - xacro

## Windows 与 Linux 之间的 USB 设备共享

如果你在 Windows 中使用 WSL 或虚拟机运行 ROS2，需要将 USB 设备从 Windows 共享到 Linux。

### Windows 主机设置：

1. 使用管理员权限打开 PowerShell 终端：

   ```powershell
   # 安装 usbipd
   winget install usbipd
   
   # 列出所有 USB 设备
   usbipd list
   ```

2. 找到 LS01G 激光雷达（通常显示为 Silicon Labs CP210x），记下其总线 ID（如 2-4）

3. 绑定并附加设备到 WSL：

   ```powershell
   # 绑定设备（替换 2-4 为你的设备 ID）
   usbipd bind 2-4 -f
   
   # 附加设备到 WSL（替换 duzhong 为你的 WSL 用户名）
   usbipd attach -b 2-4 -w duzhong
   ```

### Linux 端验证：

```bash
# 验证设备是否已连接
ls -l /dev/ttyUSB*
lsusb | grep "Silicon Labs"
```

## 设置 udev 规则

为确保激光雷达设备权限正确并且路径固定，需要设置 udev 规则：

1. 进入 ls01g 包的脚本目录：

   ```bash
   cd ~/learn/src/ls01g/scripts
   ```

2. 运行 udev 规则安装脚本：

   ```bash
   python3 install_udev.py
   sudo python3 install_udev.py # 自动安装规则
   ```

3. 确认规则安装成功：

   ```bash
   ls -l /dev/laser
   ```

   应该看到它是指向 /dev/ttyUSB0 的符号链接，权限为 rw-rw-rw-。

## 创建 ROS2 工作区

如果尚未创建工作区，按照以下步骤创建：

```bash
mkdir -p ~/learn/src
cd ~/learn/src
# 克隆 ls01g 包（如果需要）
git clone https://github.com/yourname/ls01g.git
```

## 编译

```bash
# 确保 ROS2 环境已启动
source /opt/ros/humble/setup.bash

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-joint-state-publisher ros-humble-xacro

# 进入工作区根目录
cd ~/learn

# 编译
colcon build --packages-select ls01g

# 加载环境
source install/setup.bash
```

## 运行

运行 ls01g 驱动，开始发布激光数据：

```bash
# 使用默认串口设备
ros2 launch ls01g ls01g.launch.py

# 或指定串口设备
ros2 launch ls01g ls01g.launch.py port:=/dev/laser

# 设置雷达参数
ros2 launch ls01g ls01g.launch.py port:=/dev/laser inverted:=true
```

启动后，你应该可以在 RViz 中看到激光数据和激光雷达模型。

## 功能说明

本驱动包含以下功能：

1. **数据采集与发布**：
   - 读取激光雷达数据并转换为标准 ROS2 LaserScan 消息
   - 发布到 `/scan` 主题，兼容 ROS2 导航堆栈

2. **3D 可视化**：
   - 支持在 RViz 中显示激光雷达模型
   - 实时显示激光扫描数据
   - 提供预配置的 RViz 配置文件

3. **TF 变换**：
   - 发布 `map` → `base_laser_link` 的坐标变换
   - 支持与机器人模型集成

4. **配置灵活性**：
   - 可通过启动参数自定义串口设备、雷达朝向等
   - 提供 xacro 模型支持自定义外观和尺寸

## 参数配置

可通过 launch 文件配置以下参数：

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| port | /dev/ttyUSB0 | 激光雷达串口设备 |
| inverted | false | 如果0度方向在串口线的方向上设置为true |
| scan_topic | scan | 发布的扫描数据主题名称 |
| laser_link | base_laser_link | 激光雷达坐标系名称 |
| zero_as_max | false | 设置为true时，探测不到区域会变成最大值 |
| min_as_zero | false | 设置为true时，探测不到区域为0，false为inf |
| angle_disable_min | -1.0 | 角度制，从此值到max之间的值为0 |
| angle_disable_max | -1.0 | 角度制，从min到此值之间的值为0 |

## 故障排除

### 无法找到设备

- 检查设备连接: `ls -l /dev/ttyUSB*`
- 确认 CP210x 驱动已加载: `lsmod | grep cp210x`
- 检查设备权限: `sudo chmod 666 /dev/ttyUSB0`
- 重新插拔 USB 设备并检查内核日志: `dmesg | tail`

### 编译错误

- 确保所有依赖已安装: `rosdep install --from-paths src --ignore-src -r -y`
- 检查 CMakeLists.txt 中的库路径是否正确
- 检查构建错误: `colcon build --packages-select ls01g --event-handlers console_direct+`

### 段错误或启动失败

- 检查串口设备是否已连接: `ls -l /dev/laser`
- 确保用户在 dialout 组中: `sudo usermod -a -G dialout $USER` (需要注销并重新登录)
- 检查共享库是否正确加载: `ldd ~/learn/install/ls01g/lib/ls01g/ls01g_node`

### 在 RViz 中看不到模型

- 确认 URDF/Xacro 文件存在并正确加载: `ls -l ~/learn/install/ls01g/share/ls01g/xacro/`
- 检查 TF 树是否正确: `ros2 run tf2_tools view_frames`
- 尝试在 RViz 中手动添加和配置 RobotModel 显示
- 如果使用 xacro，确保已安装 xacro 包: `sudo apt install ros-humble-xacro`

### 数据异常

- 检查波特率是否正确（默认为 115200 或 230400）
- 检查 `inverted` 参数是否需要调整
- 检查激光雷达是否正常旋转
- 使用串口监控工具检查原始数据: `sudo apt install gtkterm && gtkterm -p /dev/ttyUSB0 -s 115200`

## 开发指南

### 代码结构

- `include/` - 头文件
- src - 源代码
  - `ls01g_driver.cpp` - 激光雷达驱动库
  - `ls01g_node.cpp` - ROS2 节点实现
- `launch/` - 启动文件
- `xacro/` - 机器人描述文件
- `meshes/` - 3D 模型文件
- `rviz/` - RViz 配置文件

### 调试技巧

1. 使用 GDB 调试节点:
   ```bash
   ros2 run --prefix 'gdb -ex run --args' ls01g ls01g_node
   ```

2. 查看发布的话题:
   ```bash
   ros2 topic list
   ros2 topic echo /scan
   ```

3. 查看节点参数:
   ```bash
   ros2 param list
   ros2 param get /ls01g_node scan_topic
   ```

4. 记录和回放数据:
   ```bash
   ros2 bag record /scan
   ros2 bag play your_recorded_bag
   ```

## 许可说明

该软件包基于 MIT 许可证发布。详情请参阅 LICENSE 文件。

---

本驱动基于乐动机器人 LS01G 激光雷达规格开发，支持 ROS2 Humble 及更高版本。有问题或建议请在 GitHub 上创建 Issue。