# 從零建置機器人與相機環境（ROS 2 Jazzy）

本講義說明如何建置本專案使用的 ROS 2、Python、機械手臂與相機環境。以下指令與設定以 Ubuntu 24.04、ROS 2 Jazzy、UR3 和 Robotiq 夾爪為準；若使用其他設備，請調整網路位址與硬體相關參數。

英文版：[ur3_ros2_setup.md](ur3_ros2_setup.md)

## 目錄

- [硬體設定](#hardware-configuration)
- [1. 安裝 ROS 2](#install-ros-2)
- [2. 安裝機械手臂、相機與 MoveIt 套件](#install-robot-camera-and-moveit-packages)
- [3. 建立工作區](#create-a-workspace)
- [4. 下載原始碼並建置](#download-source-and-build)
- [5. 確認 UR3 與夾爪網路連線](#check-ur3-and-gripper-network-access)
- [6. 啟動系統](#launch-commands)
- [7. 檢查 driver 與 MoveIt](#check-the-driver-and-moveit)
- [筆記型電腦專屬設定](#laptop-specific-settings)
- [ROS 2 工具](#ros-2-tools)

<a id="hardware-configuration"></a>
## 硬體設定

| 項目 | 設定 |
| --- | --- |
| 作業系統 | Ubuntu 24.04 |
| ROS 版本 | ROS 2 Jazzy |
| UR 機型 | UR3 |
| UR3／夾爪 IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt 規劃群組 | `manipulator` |

若使用其他硬體，請依設備調整 shell 指令與 Python 程式中的參數。

<a id="install-ros-2"></a>
## 1. 安裝 ROS 2

本指南以 Ubuntu 24.04 與 ROS 2 Jazzy 為例。若使用其他 ROS 2 發行版，請將以下指令中的 `jazzy` 替換成該發行版名稱。

確認系統使用 UTF-8 locale：

```bash
locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

啟用必要的軟體來源，並安裝加入 ROS 軟體來源所需的工具：

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

加入 ROS 2 軟體來源並更新套件清單：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
```

安裝 ROS 2 Jazzy Desktop 與開發工具，包含 colcon、rosdep 和 vcstool：

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

每次開啟新終端都要載入 ROS 2 環境：

```bash
source /opt/ros/jazzy/setup.bash
```

若要讓新終端自動載入，可將指令加入 `~/.bashrc`：

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

<a id="install-robot-camera-and-moveit-packages"></a>
## 2. 安裝機械手臂、相機與 MoveIt 套件

安裝硬體抽象層與控制器套件：

```bash
sudo apt install -y ros-jazzy-ros2-control
sudo apt install -y ros-jazzy-ros2-controllers
sudo apt install -y ros-jazzy-control-msgs
sudo apt install -y ros-jazzy-trajectory-msgs
```

安裝運動規劃與視覺化套件：

```bash
sudo apt install -y ros-jazzy-moveit
sudo apt install -y ros-jazzy-rviz2
sudo apt install -y ros-jazzy-robot-state-publisher
sudo apt install -y ros-jazzy-moveit-py
```

以下二進位 UR 套件為選用；本指南後續會從原始碼建置 UR 套件：

```bash
sudo apt update
sudo apt install -y ros-jazzy-ur
```

安裝程式使用的 Python 套件：

```bash
sudo apt install -y python3-numpy python3-yaml python3-pip
```

### RealSense SDK 與工具

若目前 apt 軟體來源沒有提供所需的 RealSense 套件，請加入 RealSense 軟體來源並安裝 SDK：

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

<a id="create-a-workspace"></a>
## 3. 建立工作區

請在每台電腦建立工作區。建議路徑為 `~/ur3_ws`，也可以選擇其他路徑。

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ur3_ws/src
cd ~/ur3_ws
```

<a id="download-source-and-build"></a>
## 4. 下載原始碼並建置

將 Universal Robots driver 與 RealSense ROS 2 wrapper 下載到工作區：

```bash
cd ~/ur3_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

### 將共用虛擬桌面加入 MoveIt

將共用場景套件複製到工作區：

```bash
cp -r /path/to/robotic\ control/ur3_moveit_scene ~/ur3_ws/src/
```

在 `~/ur3_ws/src/Universal_Robots_ROS2_Driver` 的 Universal Robots driver 原始碼中，將此套件整合到 MoveIt：

1. 在 `ur_moveit_config/package.xml` 加入 `<exec_depend>ur3_moveit_scene</exec_depend>`。
2. 在 `ur_moveit_config/launch/ur_moveit.launch.py` 中，設定 MoveIt 啟動時一併啟動 `ur3_moveit_scene` 套件提供的 `virtual_table` 執行程式。目前專案中的工作區副本已包含這些修改。

虛擬桌面是 `base_link` 座標系中的水平碰撞板，尺寸為 **4 m × 4 m**。上表面高度由 `virtual_table_z` 設定，預設為 `-0.01 m`；板厚為 **0.02 m**，並從上表面往下延伸。例如，`virtual_table_z:=-0.04` 會將虛擬桌面的上表面設在 `z = -0.04 m`，碰撞板範圍則是 `z = -0.06 m` 到 `-0.04 m`。請量測實體桌面相對於 `base_link` 的高度並設定此參數；launch 不會自動量測桌面位置。

MoveIt 啟動時，若 `virtual_table_enabled:=true`（預設值），MoveIt launch 會啟動桌面發布節點。節點透過 MoveIt 的 `/apply_planning_scene` 服務加入碰撞物件，再透過 `/planning_scene` 發布場景更新，讓已連線的場景監控器收到桌面資料。節點也會在 `/moveit_virtual_table/marker` 發布 RViz 顯示用標記。碰撞檢查使用的是 MoveIt Planning Scene 中的碰撞物件，不是 RViz 標記本身。使用同一個 MoveIt 場景的規劃程式都能檢查這張桌面，不限於 pick 程式。

Easy HandEye 2 的安裝與自動校正程式整合方式，請參閱 [Easy HandEye 2 指南](handeye/README.md)第 2 節。接著依照下方步驟安裝相依套件並建置工作區。

若此系統尚未初始化 rosdep，請先執行初始化，再更新套件索引：

```bash
sudo rosdep init
rosdep update
```

安裝相依套件並建置工作區：

```bash
cd ~/ur3_ws
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
source ~/ur3_ws/install/setup.bash
```

MoveIt 設定整合 `ur3_moveit_scene` 後，可執行以下指令重新建置相關套件：

```bash
colcon build --packages-up-to ur_moveit_config --symlink-install
```

若要讓新終端自動載入工作區，將以下指令加入 `~/.bashrc`：

```bash
source ~/ur3_ws/install/setup.bash
```

<a id="check-ur3-and-gripper-network-access"></a>
## 5. 確認 UR3 與夾爪網路連線

以下系統層級檢查可確認筆記型電腦能否連到機械手臂與夾爪服務。本指南使用 UR3 IP `192.168.86.7`。

確認 UR3 是否回應 ICMP：

```bash
ping -c 3 192.168.86.7
```

若沒有回應，請檢查網路設定與設備連線。有些網路會封鎖 ICMP；也請確認筆記型電腦連到正確的網路、IP 位址位於可連線的子網路，且 UR3 已開機並連線。

確認 Robotiq 的 TCP socket（port `63352`）是否可連線：

```bash
nc -vz 192.168.86.7 63352
```

若系統沒有 `nc`，請安裝 `netcat-openbsd`：

```bash
sudo apt install netcat-openbsd
```

連線成功只代表該 port 可連線，不代表 ROS 2 或夾爪功能已正常。若連線失敗，請確認 UR 控制器與 Robotiq socket server 正在運作、IP 與 port 正確，且網路允許此連線。

<a id="launch-commands"></a>
## 6. 啟動系統

建議在不同終端分別啟動各項服務，方便查看各自的 log，也能個別停止或重新啟動。每個終端都要載入 ROS 2 與工作區：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

**終端 1：啟動實體 UR3 driver，並關閉 driver 自己的 RViz：**

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.86.7 use_mock_hardware:=false launch_rviz:=false
```

若要使用 Mock Hardware，將 `use_mock_hardware:=false` 改成 `use_mock_hardware:=true`。此模式不會連接實體 UR3。

**終端 2：啟動 MoveIt、RViz 與共用虛擬桌面：**

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true virtual_table_enabled:=true virtual_table_z:=-0.01
```

將 `virtual_table_z` 設為量測到的實體桌面在 `base_link` 座標系中的高度。只有在需要不載入碰撞桌面時，才設定 `virtual_table_enabled:=false`。整合式 bringup launch 也會把這兩個參數傳給 MoveIt，例如：

```bash
ros2 launch ur3_bringup bringup.launch.py use_mock_hardware:=true virtual_table_z:=-0.04
```

此例使用 Mock Hardware，並將虛擬桌面的上表面設在 `base_link` 的 `z = -0.04 m`。

**終端 3：啟動 RealSense 相機：**

```bash
ros2 launch realsense2_camera rs_launch.py
```

若要在其他終端執行 Python 程式：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
python3 ~/ur3_ws/src/<script_name>.py
```

操作實體手臂前，請先開啟手臂電源，並將 teach pendant 設定為可接受 External Control。筆記型電腦與機械手臂必須在同一個網路。RealSense 請連接到 USB 3.x。按下某個終端的 `Ctrl+C` 可停止該終端啟動的服務。使用整合式 bringup 時，不要同時再啟動另一組相同服務，以免重複啟動節點。

### 選用：自動開啟三個終端視窗

專案附有 helper script，可開啟三個 GNOME Terminal 視窗，並在每個視窗啟動一項服務。第一次使用時，將旁邊完整的 `ur3_bringup/` 套件複製到 `~/ur3_ws/src/`，包含 `package.xml`、`CMakeLists.txt`、`launch/` 和 `scripts/`。依照第 4 節下載原始碼後，安裝相依套件並建置：

```bash
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

確認輸出包含 `Finished <<< ur3_bringup`。接著在桌面終端執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_bringup start_separate_terminals.sh
```

此 script 預設 UR IP 為 `192.168.86.7`，並讓 RealSense driver 自行選擇相機。可用第二個參數選擇是否使用 Mock Hardware；預設 `false` 會連接實體手臂。參數格式為 `[UR_IP] [true|false]`：

```bash
ros2 run ur3_bringup start_separate_terminals.sh 192.168.86.7 true
```

此例會以 Mock Hardware 啟動 UR driver，不會連接實體 UR3；MoveIt 與相機仍會啟動。若要控制實體手臂，請省略第二個參數或設為 `false`。通常不需要指定相機序號，這個 script 也沒有提供相機序號參數。

每個視窗會顯示該服務的 log。在視窗中按下 `Ctrl+C` 停止服務，再按 Enter 關閉視窗。此方法需要 GNOME Terminal；若尚未安裝，可執行 `sudo apt install gnome-terminal`。複製或更新 `ur3_bringup` 後，請執行 `colcon build --symlink-install` 重新建置並重新載入工作區。

<a id="check-the-driver-and-moveit"></a>
## 7. 檢查 driver 與 MoveIt

檢查軌跡控制器：

```bash
ros2 topic list | grep joint_trajectory_controller
```

檢查 MoveIt IK 服務：

```bash
ros2 service list | grep compute_ik
```

列出 ROS 2 nodes、topics 與 services：

```bash
ros2 node list
ros2 topic list
ros2 service list
```

<a id="laptop-specific-settings"></a>
## 筆記型電腦專屬設定

每台筆記型電腦都要分別設定工作區路徑、`.bashrc` 載入路徑與網路介面卡。如果所有電腦都使用 `~/ur3_ws`，工作區相關指令可以相同。

<a id="ros-2-tools"></a>
## ROS 2 工具

可使用 `rqt_image_view` 查看 ROS 2 發布的相機影像：

```bash
ros2 run rqt_image_view rqt_image_view
```

