# UR3 + Robotiq 控制環境安裝參考

因此：
- **手臂本身的固定設定可以直接沿用**
- **每台筆電自己的路徑、Python 環境與 workspace 需自行建立**

---

## 共用硬體設定

| 項目 | 設定 |
| --- | --- |
| 作業系統建議 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| UR 型號 | UR3 |
| UR3 / Gripper IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt planning group | `manipulator` |

> 因為大家使用的是同一台 UR3，所以 `192.168.86.7` 與 `63352` 可以直接使用，不需要改成各自不同的值。

---

## 必要 ROS 套件

| 套件 | 用途 |
| --- | --- |
| `ur_robot_driver` | 與 UR3 建立通訊、接收 MoveIt 軌跡並發布機器人狀態 |
| `ur3_moveit_config` | UR3 的 MoveIt、IK、OMPL、controller 等設定 |
| `moveit_commander` | Python MoveIt API |
| `moveit_msgs` | MoveIt message、IK service 與 error code |
| `moveit_planners_ompl` | MoveIt OMPL 路徑規劃 |
| `ur_client_library` | Universal Robots ROS Driver 底層通訊 library |
| `rospy` | Python ROS node、topic、service、parameter |
| `geometry_msgs` | Pose、Point、Quaternion 等幾何訊息 |
| `std_msgs` | ROS 標準訊息 |
| `tf` / `tf2_ros` / `tf2_geometry_msgs` | 座標系與 Pose 轉換 |
| `visualization_msgs` | RViz marker 等視覺化訊息 |

## 必要 Python 套件

| 套件 | 用途 |
| --- | --- |
| `numpy` | Pose、rotation、軌跡與幾何運算 |
| `PyYAML` | 讀取 YAML 參數檔 |

Robotiq 夾爪若使用專案內的 TCP socket 控制方式，只需要 Python standard library：

- `socket`
- `threading`
- `time`
- `enum`
- `typing`

因此 **不需要另外安裝 Robotiq ROS package**。

---

## 1. 安裝 ROS、MoveIt 與 Python 依賴

以下以 Ubuntu 20.04 + ROS Noetic 為基準。

```bash
sudo apt update

sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-moveit-commander \
  ros-noetic-moveit-msgs \
  ros-noetic-moveit-planners-ompl \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-geometry-msgs \
  ros-noetic-std-msgs \
  ros-noetic-visualization-msgs \
  ros-noetic-ur-client-library \
  python3-numpy \
  python3-yaml \
  python3-pip \
  python3-rosdep \
  python3-vcstool
```

載入 ROS：

```bash
source /opt/ros/noetic/setup.bash
```

---

## 2. 建立 UR3 Driver Workspace

每台筆電都需要自己建立 workspace。

以下使用 `~/ur3_ws` 作為建議名稱，也可以自行更換。

```bash
source /opt/ros/noetic/setup.bash

mkdir -p ~/ur3_ws/src
cd ~/ur3_ws
```

下載 Universal Robots ROS Driver：

```bash
git clone --branch v2.4.1 \
  https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git \
  src/ur_robot_driver
```

下載 Universal Robot MoveIt 設定：

```bash
git clone --branch noetic-devel \
  https://github.com/ros-industrial/universal_robot.git \
  src/universal_robot
```

安裝相依套件：

```bash
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -r -y
```

編譯：

```bash
catkin_make
```

載入 workspace：

```bash
source ~/ur3_ws/devel/setup.bash
```

如果每次開 terminal 都要使用，也可以自行加入 `~/.bashrc`：

```bash
source /opt/ros/noetic/setup.bash
source ~/ur3_ws/devel/setup.bash
```

---

## 3. UR3 網路連線檢查

筆電需要和 UR3 位於可以互相連線的網路環境。

UR3 IP 固定為：

```text
192.168.86.7
```

測試：

```bash
ping -c 3 192.168.86.7
```

如果 ping 不到，先檢查：

- 筆電是否連到正確網路
- 筆電網卡 IP 是否與 UR3 位於可互通網段
- UR3 是否已開機並連線

---

## 4. Robotiq Gripper 連線檢查

本專案使用 UR 控制器上的 Robotiq socket server。

Port：

```text
63352
```

測試：

```bash
nc -vz 192.168.86.7 63352
```

如果沒有 `nc`：

```bash
sudo apt install netcat-openbsd
```

成功時代表筆電可以連到 Robotiq socket server。

---

## 5. ROS 套件安裝檢查

```bash
rospack find ur_robot_driver
rospack find ur3_moveit_config
rospack find moveit_commander
```

確認 Python 可以 import 控制程式需要的套件：

```bash
python3 -c "import rospy, moveit_commander, moveit_msgs, tf2_ros, tf2_geometry_msgs, numpy, yaml; print('OK')"
```

---

## 6. Driver / MoveIt 啟動後檢查

確認 trajectory controller：

```bash
rostopic list | grep scaled_pos_joint_traj_controller
```

確認 MoveIt IK service：

```bash
rosservice list | grep compute_ik
```

查看 ROS node：

```bash
rosnode list
```

查看 ROS topic：

```bash
rostopic list
```

查看 ROS service：

```bash
rosservice list
```

---

## 每台筆電需要自行設定的部分

以下內容不要直接複製其他人的設定：

- `/home/<username>/...` 路徑
- workspace 實際位置
- Conda / virtualenv 路徑
- `~/.local/...` Python package 路徑
- 個人 `.bashrc` 內的 source 路徑
- 個人電腦的網卡設定

如果大家都使用 `~/ur3_ws`，則 workspace 指令可以完全一致。

---

## 可以直接共用的設定

因為使用的是同一台實體 UR3，所以以下可以直接共用：

```text
UR 型號            UR3
UR IP              192.168.86.7
Robotiq Port       63352
MoveIt Group       manipulator
ROS                Noetic
Ubuntu             20.04
```

程式碼中如果有寫死：

```python
ROBOT_IP = "192.168.86.7"
```

或：

```python
RobotiqGripper("192.168.86.7", 63352)
```

只要連的是這台 UR3，就不需要修改。
