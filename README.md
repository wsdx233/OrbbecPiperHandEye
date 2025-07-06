### 1. 项目文件结构
建议在工作空间中创建一个新的ROS2包，例如`hand_eye_calibration`。

```
<your_ros2_ws>/src/
├── OrbbecSDK_ROS2-main/
├── piper_ros-humble/
└── hand_eye_calibration/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    └── hand_eye_calibration/
        ├── __init__.py
        └── aruco_hand_eye_calibration_node.py
```

### 2. 依赖项

在开始之前，请确保您已经安装了必要的Python库：

```bash
pip3 install opencv-python opencv-contrib-python numpy scipy
```


### 3. 如何使用

#### 步骤 1: 编译

在您的ROS2工作空间根目录运行 `colcon build`：
```bash
cd <your_ros2_ws>
colcon build --packages-select hand_eye_calibration
source install/setup.bash
```

#### 步骤 2: 运行系统
您需要同时运行Orbbec相机节点和Piper机械臂的驱动节点。

**终端 1: 启动 Orbbec 相机**
```bash
# source <orbbec_ws>/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py # 或适用于您相机的launch文件
```

**终端 2: 启动 Piper 机械臂**
```bash
# source <piper_ws>/install/setup.bash
bash can_activate.sh can0 1000000
ros2 launch piper start_single_piper_rviz.launch.py
```
**注意**: 在`start_single_piper_rviz.launch.py`中，RViz会发布`/joint_states`话题，这个话题会被`piper_single_ctrl`节点接收来控制真实的机械臂。你需要**拖动RViz中的滑块**来移动机械臂。

#### 步骤 3: 运行标定节点
在新的终端中，启动我们刚刚创建的手眼标定节点。

```bash
ros2 run hand_eye_calibration aruco_calibrator --ros-args -p marker_size:=0.05
```
*   `marker_size`: **非常重要**，请确保设置为您打印的ArUco标记的实际边长（单位：米）。
*   您可以根据需要通过`-p`参数覆盖其他默认值，例如`base_frame`、`end_effector_frame`等。

#### 步骤 4: 实时查看
您可以使用`rqt_image_view`来查看调试图像。
```bash
rqt_image_view /aruco_detect/image_raw
```
您应该能看到相机视野，并且当ArUco标记出现时，它会被一个绿色的框和三维坐标轴标记出来。

#### 步骤 5: 采集数据
这是标定过程的核心。

1.  将ArUco标记**固定**在Piper机械臂的末端执行器上（例如，用胶带粘在夹爪上）。
2.  在RViz中，拖动滑块或使用MoveIt（如果您配置了）将机械臂移动到一个新的位置和姿态。确保ArUco标记清晰地出现在相机的视野中。
3.  打开一个新的终端，调用`capture_pose`服务：
    ```bash
    ros2 service call /capture_pose std_srvs/srv/Trigger
    ```
4.  您应该会在标定节点的终端中看到一条消息，提示已成功捕获一个位姿对，并显示当前已捕获的总数。
5.  **重复步骤2和3至少5-10次**。每次都将机械臂移动到**不同**的位置和姿态。姿态的多样性（不同的旋转角度）对于获得准确的标定结果至关重要。

#### 步骤 6: 计算标定结果
当您采集了足够多的数据点后，调用`calculate_calibration`服务：
```bash
ros2 service call /calculate_calibration std_srvs/srv/Trigger
```
节点将执行计算，并在其终端窗口中打印出详细的标定结果，包括4x4变换矩阵、平移向量和四元数/欧拉角。

#### 步骤 7: 应用标定结果
将打印出的平移和旋转（通常使用四元数）应用到您的TF配置中，通常是一个静态TF发布器，来发布`base_link`到`camera_link`（或`camera_color_optical_frame`）的变换。

例如，在您的launch文件中添加一个`static_transform_publisher`节点：
```python
# In your launch file
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_camera_broadcaster',
    # Replace arguments with your calibration result
    # arguments = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'parent_frame', 'child_frame']
    arguments=['0.5', '0.0', '0.5', '0.0', '0.0', '0.0', '1.0', 'base_link', 'camera_link']
)
```
