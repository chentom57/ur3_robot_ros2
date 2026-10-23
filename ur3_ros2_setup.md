## 從零開始建構手臂與相機環境（ROS2 Jazzy）

**此講義為建立機械手臂控制所需環境之教學，包含建立ROS2,Python,手臂與相機環境**

## 📌 文件大綱 (Outline)

* [硬體設定](#硬體設定)
* [1. ROS2 安裝教學](#1-ros2-安裝教學)
* [2. ROS2 相關套件安裝（手臂、相機與 MoveIt 2）](#2-ros2-相關套件安裝手臂相機與-moveit-2)
* [3. 專屬Workspace建立](#3-專屬workspace建立)
* [4. 原始碼下載與編譯](#4-原始碼下載與編譯)
* [5. UR3 與 Robotiq Gripper 基礎網路連線檢查](#5-ur3-與-robotiq-gripper-基礎網路連線檢查)
* [6. 啟動指令](#6-啟動指令)
* [7. Driver / MoveIt 檢查指令](#7-driver--moveit-檢查指令)
* [每台筆電需要自行設定的部分](#每台筆電需要自行設定的部分)
* [ROS2 內建工具](#ros2-內建工具)

---

## 硬體設定
UR3手臂硬體設定:
| 項目 | 設定 |
| --- | --- |
| 作業系統 | Ubuntu 24.04 |
| ROS版本 | ROS2 Jazzy |
| UR 型號 | UR3 |
| UR3 / Gripper IP | `192.168.86.7` |
| Robotiq socket port | `63352` |
| MoveIt planning group | `manipulator` |

若使用不同硬體請適當更換 bash 指令和 python3 程式內參數.


## 1. ROS2 安裝教學

以下以 Ubuntu 24.04 + ROS2 jazzy為例,若要改其他ROS2板,請將6️⃣後的`jazzy`改成其他版本

1️⃣ 確保系統支援 UTF-8 語系：

```bash
locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
2️⃣ 啟用必需的存儲庫

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
3️⃣ 使用apt添加 ROS 2 GPG 金鑰

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
4️⃣ 將存儲庫添加到您的源清單中
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
5️⃣ 更新系統的軟體源清單 

```bash
sudo apt update
sudo apt upgrade
```
6️⃣ 安裝 ROS 2 Jazzy 完整桌面版 與 開發工具箱（內含colcon、rosdep、vcstool...）

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

7️⃣ 載入 ROS：

每次開terminal時,須執行以下指令載入ROS2：
```bash
source /opt/ros/jazzy/setup.bash
```
不想每次開terminal後都載入，可以寫入` ~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```
---
## 2. ROS2 相關套件安裝（手臂、相機與 MoveIt 2）

硬體抽象層與控制器模組:

```bash
# 安裝 ros2_control 硬體抽象層核心
sudo apt install -y ros-jazzy-ros2-control

# 安裝官方內建的機械臂控制器算法（如關節軌跡控制）
sudo apt install -y ros-jazzy-ros2-controllers

# 安裝控制系統專用的標準通訊格式
sudo apt install -y ros-jazzy-control-msgs

# 安裝機械臂連續移動軌跡的資料格式
sudo apt install -y ros-jazzy-trajectory-msgs

```

運動規劃與視覺化相關模組:

```bash
# 安裝 MoveIt 2 核心運動規劃庫
sudo apt install -y ros-jazzy-moveit

# 安裝 RViz2 3D 視覺化模擬視窗
sudo apt install -y ros-jazzy-rviz2

# 安裝機器人狀態與 TF 空間座標發布器
sudo apt install -y ros-jazzy-robot-state-publisher

# 安裝 MoveIt 2 的 Python API 介面
sudo apt install -y ros-jazzy-moveit-py
```

UR系列手臂官方驅動程式(非必要,下面會安裝原始碼版本):

```bash
# 安裝 ur_robot_driver 、 ur_controllers 、 ur_description 、ur_moveit_config 等核心組件
sudo apt update
sudo apt install -y ros-jazzy-ur
``` 

python3相關套件：
```bash
sudo apt install -y python3-numpy \
  python3-yaml \
  python3-pip \
```
以下是當需要工具不在 apt 裡時，所需的安裝流程：

1. RealSense 2（相機驅動）

   1️⃣ 建立資料夾存放 apt 套件庫驗證金鑰。

   ```bash
   sudo mkdir -p /etc/apt/keyrings
   ```

   2️⃣ 下載並安裝 RealSense 官方套件庫的驗證金鑰。

   ```bash
   curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
     | gpg --dearmor \
     | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null
   ```

   3️⃣ 將 RealSense 官方套件庫加入 apt。

   ```bash
   echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
     | sudo tee /etc/apt/sources.list.d/librealsense.list
   ```

   4️⃣ 安裝 RealSense SDK 和相關工具。

   ```bash
   sudo apt update
   sudo apt install -y \
     librealsense2-dkms \
     librealsense2-utils \
     librealsense2-dev
   ```
---

## 3. 專屬Workspace建立 

每台筆電都需要自己建立 workspace，隔離外部環境維持安全性。

以下使用 `~/ur3_ws` 作為建議名稱(可自行變更):

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/ur3_ws/src
cd ~/ur3_ws
```
---
## 4. 原始碼下載與編譯

在建好的 workspace 中下載 UR 系列官方驅動和 RealSense ROS 2 wrapper：
```bash
cd ~/ur3_ws/src
# 1. 下載 Universal Robots 驅動原始碼
git clone -b jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
# 2. 下載 RealSense ROS 2 wrapper 原始碼
git clone -b ros2-master https://github.com/realsenseai/realsense-ros.git
```

Easy HandEye 2 的安裝與自動校正程式整合方式請參考 [Easy HandEye 2 手眼標定講義](handeye/easy_handeye2_guide.md) 第 2 節；完成後再執行本節的 `rosdep install` 和 `colcon build`。

若第一次使用`rosdep`則要初始化:
```bash 
sudo rosdep init
```
更新相依套件索引:
```bash
rosdep update 
```

掃描並安裝所需的相依套件：
```bash
cd ~/ur3_ws
rosdep install -i --from-paths src --rosdistro jazzy -y
```

編譯：

```bash
cd ~/ur3_ws
colcon build --symlink-install
```

載入 workspace：

```bash
source ~/ur3_ws/install/setup.bash
```

跟 ROS2 一樣，若不想每次開 terminal 都要使用，也可以自行加入 `~/.bashrc`：

```bash
source ~/ur3_ws/install/setup.bash >> ~/.bashrc
```
---


## 5. UR3 與 Robotiq Gripper 網路連線檢查

以下是作業系統層級的網路檢查，以確認筆電與手臂＋夾爪位於可互相連線的網路環境。

本文件使用的 UR3 IP 為 `192.168.86.7`。

### 檢查 UR3 是否可透過網路連線

使用 `ping` 檢查 UR3 是否回應 ICMP 封包：

```bash
ping -c 3 192.168.86.7
```

若沒有回應，可能是網路設定或設備連線有問題；有些網路也會封鎖 ICMP。可再檢查：

- 筆電是否連到正確網路
- 筆電網卡 IP 是否與 UR3 位於可互通網段
- UR3 是否已開機並連線



### 檢查 Robotiq socket port

本專案使用 UR 控制器上的 Robotiq socket server。以 `nc` 確認 TCP port `63352` 是否可連線：

```bash
nc -vz 192.168.86.7 63352
```

如果沒有 `nc`：

```bash
sudo apt install netcat-openbsd
```

若連線成功，代表該 TCP port 可從筆電連到；這只確認網路與 socket port 可達，不代表 ROS 2 或夾爪控制功能已通過測試。若連線失敗，請確認 UR 控制器與 Robotiq socket server 已啟動、IP 與 port 設定正確，且網路未封鎖該連線。

---

## 6. 啟動指令

請使用多個 terminal 分別啟動服務。此方式可個別檢視 log、停止或重啟節點。每個新 terminal 都先載入 ROS 2 和 workspace 環境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

Terminal 1（實體 UR3 driver；driver 自己的 RViz 關閉）：

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.86.7 use_mock_hardware:=false launch_rviz:=false
```

若要改用模擬硬體，將 `use_mock_hardware:=false` 改成 `use_mock_hardware:=true`；此時不會連接實體 UR3。

Terminal 2（MoveIt 與 RViz）：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

Terminal 3（RealSense 相機）：

```bash
ros2 launch realsense2_camera rs_launch.py
```

可再開一個 terminal 執行自己的 Python 程式：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
python3 ~/ur3_ws/src/<腳本名稱>.py
```

實體手臂需先開機、示教器進入可接受 External Control 的狀態，筆電與手臂需在同一網路；RealSense 需先接上 USB 3.x。每個 terminal 按 `Ctrl+C` 會停止該 terminal 啟動的服務。不要再同時啟動整合 launch，避免重複啟動節點。

### 自動開啟三個 terminal 視窗（可選）

如果不想手動在三個 terminal 貼上啟動指令，可使用附帶的 helper script 自動開啟三個 GNOME Terminal 視窗，各自在獨立視窗啟動一個服務。首次使用需先將講義資料夾旁附的整個 `ur3_bringup/`（包含 `package.xml`、`CMakeLists.txt`、`launch/` 和 `scripts/`）複製到 `~/ur3_ws/src/`；完成第 4 節下載官方原始碼後，執行：

```bash
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

確認輸出中有 `Finished <<< ur3_bringup`。之後從桌面圖形環境的 terminal 執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_bringup start_separate_terminals.sh
```

此 script 會使用預設 UR IP `192.168.86.7`，且讓 RealSense driver 自動選取相機。第二個參數可選擇是否使用 mock hardware；預設 `false` 會連接實體手臂。參數格式為 `[UR_IP] [true|false]`：

```bash
ros2 run ur3_bringup start_separate_terminals.sh 192.168.86.7 true
```

上例以 mock hardware 啟動 UR driver，不會連接實體 UR3；MoveIt 和相機仍會啟動。若要控制實體手臂，省略第二個參數或明確指定 `false`。相機序號通常不需要設定，故不列為啟動參數。

每個視窗可單獨看 log，並在該視窗按 `Ctrl+C` 停止對應服務；服務停止後，按 Enter 關閉視窗。這個方式需要安裝 `gnome-terminal`（若沒有，執行 `sudo apt install gnome-terminal`）。如果剛複製/更新 `ur3_bringup`，先重新執行 `colcon build --symlink-install`，再 source workspace。

---
## 7. Driver / MoveIt 檢查指令

確認 trajectory controller：

```bash
ros2 topic list | grep joint_trajectory_controller
```

確認 MoveIt IK service：

```bash
ros2 service list | grep compute_ik
```

查看 ROS2 node：

```bash
ros2 node list
```

查看 ROS2 topic：

```bash
ros2 topic list
```

查看 ROS2 service：

```bash
ros2 service list
```

---

## 每台筆電需要自行設定的部分

- workspace 實際位置
- 個人 `.bashrc` 內的 source 路徑
- 個人電腦的網卡設定

如果大家都使用 `~/ur3_ws`，則 workspace 指令可以完全一致。

---

## ROS2 內建工具

rqt(用於讀取相機發布的影像)：

```bash
ros2 run rqt_image_view rqt_image_view
```

---

