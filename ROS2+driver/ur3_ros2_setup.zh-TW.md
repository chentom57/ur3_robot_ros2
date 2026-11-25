# 從零建置機器人與相機環境（ROS 2 Jazzy）

本講義說明如何建置本專案使用的 ROS 2、Python、機械手臂與相機環境。以下指令與設定以 Ubuntu 24.04、ROS 2 Jazzy、UR3 和 Robotiq 夾爪為準；若使用其他設備，請調整網路位址與硬體相關參數。

英文版：[ur3_ros2_setup.md](ur3_ros2_setup.md)

## 目錄

- [硬體設定](#hardware-configuration)
- [1. 安裝 ROS 2](#install-ros-2)
- [2. 準備主機工具與 RealSense SDK](#install-robot-camera-and-moveit-packages)
- [3. 建立工作區](#create-a-workspace)
- [4. 下載原始碼並建置](#download-source-and-build)
- [5. 確認 UR3 與夾爪網路連線](#check-ur3-and-gripper-network-access)
- [6. 分別啟動相機、UR3 與 MoveIt](#launch-commands)
- [7. 檢查 driver 與 MoveIt](#check-the-driver-and-moveit)


<a id="hardware-configuration"></a>
## 硬體設定

| 項目 | 設定 |
| --- | --- |
| 作業系統 | Ubuntu 24.04 |
| ROS 版本 | ROS 2 Jazzy |
| UR 機型 | UR3 |
| UR3／夾爪 IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt 規劃群組 | `ur_manipulator` |

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
sudo apt update && sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

加入 ROS 2 軟體來源並更新套件清單：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

安裝 ROS 2 Jazzy Desktop 與開發工具，包含 colcon、rosdep 和 vcstool：

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

每次開啟新終端都要載入 ROS 2 環境：

```bash
source /opt/ros/jazzy/setup.bash
```

<a id="install-robot-camera-and-moveit-packages"></a>
## 2. 準備主機工具與 RealSense SDK

本指南採用官方 UR driver 的 GitHub 原始碼。第 4 節會用 `rosdep` 一次安裝控制器、訊息、MoveIt、RViz、ArUco、Qt／rqt 與 Python 相依，這裡不另外安裝整套 UR 二進位套件。

先安裝下載原始碼的工具：

```bash
sudo apt install -y git
```

### RealSense SDK

若目前 apt 軟體來源沒有提供所需的 RealSense 套件，請加入 RealSense 軟體來源並安裝 SDK：

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
  | gpg --dearmor \
  | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-dev
```

本專案統一使用這套系統 SDK。原始碼版 RealSense ROS 4.59.x 需要 SDK 2.59.0 以上，目前環境使用 2.59.1。後續 `rosdep` 指令跳過 `librealsense2` 鍵，避免透過 `ros-jazzy-librealsense2` 再裝一套 SDK。請先完成系統 SDK 安裝；跳過該鍵本身不會安裝 SDK。既有環境的檢查方式見 [bringup SDK 說明](../ur3_bringup/README.zh-TW.md)。

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

將官方 UR driver（含 `ur_moveit_config`）與 RealSense ROS 4.59.1 原始碼下載到工作區。以下 clone 指令供首次建置使用，已有來源時沿用，不要重複下載：

```bash
cd ~/ur3_ws/src
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone --branch 4.59.1 --depth 1 https://github.com/realsenseai/realsense-ros.git
```

### 將完整專案放入工作區

將本專案保留一份於 `~/ur3_ws/src/robot_arm`。若先下載到其他位置，可整份複製一次，並修改下方來源路徑：

```bash
cp -r "/path/to/robot_arm" "$HOME/ur3_ws/src/robot_arm"
cd ~/ur3_ws
colcon list
```

`colcon` 會遞迴辨識 `auto_calibration`、`easy_handeye2`、`ur3_bringup`、`ur3_moveit_scene`、`ur3_pick_up` 五個套件。已將這些套件分別放入工作區的使用者可沿用原結構，不要再放第二份。不要複製其他電腦的 build／install／log。

`ur3_bringup/launch/moveit.launch.py` 會引入官方 MoveIt，再加入桌面與隨附的 `config/moveit.rviz`，不需修改官方 driver。

<a id="handeye-message-dependency"></a>
### 補入手眼校正訊息相依套件

本專案不附 `easy_handeye2_msgs`，目前這台電腦的 Jazzy rosdep 索引也沒有它的安裝規則。因此從官方來源只取出訊息套件，並固定到與目前 0.5.0 套件版本相符的 revision。若 `colcon list` 已找到一份 `easy_handeye2_msgs`，略過此步。

```bash
cd ~/ur3_ws/src
git clone --filter=blob:none --sparse \
  https://github.com/marcoesposito1988/easy_handeye2.git \
  easy_handeye2_interfaces
git -C easy_handeye2_interfaces sparse-checkout set easy_handeye2_msgs
git -C easy_handeye2_interfaces checkout --detach \
  29bd50a7939e853d768d203d2abc61eb6e50facc
```

Sparse checkout 不會取出官方的 `easy_handeye2` 主程式，可避免與本專案的 `easy_handeye2/` 重複。不要另外在同一工作區放入完整的官方主程式副本。

### 安裝相依並建置一次

五個專案套件與官方 driver 的 `package.xml` 已列出所需相依，由下方 `rosdep` 統一安裝，包含校正工具的 ArUco、MoveItPy、Qt／rqt、OpenCV、NumPy、transforms3d 與 YAML。主機不需要安裝 PyTorch 或 AnyGrasp；模型環境另依 [Docker 講義](../docker+anygrasp/README.zh-TW.md)準備。

`rosdep` 的範圍是已宣告且有安裝規則的相依。官方來源與訊息套件仍須先下載；系統 RealSense SDK 依第 2 節安裝。NVIDIA 驅動、Docker／Container Toolkit、AnyGrasp SDK、授權及模型權重，則保留在 Docker 講義中由使用者另行準備。

若此系統尚未初始化 rosdep，請先執行初始化，再更新套件索引：

```bash
sudo rosdep init
rosdep update
```

`rosdep` 讀取工作區各套件的 `package.xml`，一次安裝建置與執行相依；`--ignore-src` 會略過工作區已有來源的套件。下列只選取建置／執行相依，不安裝測試或文件相依，建置也關閉 CMake 測試。它不會下載原始碼或取代 `colcon build`。

安裝相依套件並建置工作區：

```bash
cd ~/ur3_ws
rosdep install --from-paths src --ignore-src --rosdistro jazzy \
  --skip-keys librealsense2 \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
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
## 6. 分別啟動相機、UR3 與 MoveIt

依序開啟三個終端，各自載入 ROS 與工作區並保持運作。若某個服務已啟動，沿用該服務，不要再啟動第二份。

### 終端 1：RealSense 相機與點雲

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```

RealSense 請接 USB 3.x，確認相機成功串流。只驗證相機或 ArUco 偵測時，可先只啟動此終端；tracker 的啟動方式見[手動校正講義](../easy_handeye2/README.zh-TW.md)。

### 終端 2：UR3 driver

操作實體手臂前，先開啟手臂電源並啟用 External Control；筆記型電腦與手臂須可透過網路互通。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=192.168.86.7 \
  use_mock_hardware:=false launch_rviz:=false
```

無實體手臂時，將同一指令中的 `use_mock_hardware:=false` 改成 `use_mock_hardware:=true`。相機由終端 1 獨立運作；driver 的 RViz 關閉，由終端 3 統一啟動。

### 終端 3：MoveIt、RViz 與碰撞桌面

待 UR3 driver 就緒後啟動：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true \
  virtual_table_enabled:=true virtual_table_z:=-0.01
```

這個 launch 僅整合官方 MoveIt、本專案 RViz 設定與碰撞桌面；相機及 UR3 driver 各由前兩個終端管理。桌面上表面預設為 `base_link` 的 `z=-0.01 m`，請依實際量測設定 `virtual_table_z`。完整參數見 [bringup 講義](../ur3_bringup/README.zh-TW.md)，碰撞桌面說明見 [scene 講義](../ur3_moveit_scene/README.zh-TW.md)。

自動校正與夾取依[自動校正講義](../auto_calibration/README.zh-TW.md)與[夾取講義](../ur3_pick_up/README.zh-TW.md)執行。上述三個服務已運作時，直接進入各講義的校正或夾取步驟，不再執行整合式 bringup 啟動指令。按 `Ctrl+C` 可個別停止各終端的服務。

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
