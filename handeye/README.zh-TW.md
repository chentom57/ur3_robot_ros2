# UR3 + RealSense：Easy HandEye 2 整合式手眼校正

本指南使用 `integrated_auto_handeye_calibration` 程式。腳本內含 15 個姿態的取樣、計算與儲存功能，不需要另外安裝或註冊獨立的自動校正程式。程式會先啟動 ArUco tracker 並等待偵測標記；偵測成功後詢問是否開始 Easy HandEye 2。校正儲存後，程式會詢問是否發布 TF。取樣從手臂目前姿態開始，不會先移至預設姿態。

## 硬體與軟體

| 項目 | 設定 |
|---|---|
| 作業系統 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 機械手臂 | Universal Robots UR3 |
| UR3 IP | `192.168.86.7` |
| 工作區 | `~/ur3_ws` |
| 相機 | RealSense（ROS 2 wrapper） |
| 校正標記 | ArUco `ARUCO_ORIGINAL`、ID 582、黑色方框邊長 40 mm |
| 校正套件 | `easy_handeye2` |

## 1. 選擇校正類型

| 相機安裝方式 | ArUco 板安裝方式 | 校正類型 |
|---|---|---|
| 固定在桌面或支架 | 固定在 UR3 末端工具 | `eye_on_base` |
| 固定在 UR3 末端工具 | 固定在工作區 | `eye_in_hand` |

## 2. 將整合校正程式加入 Easy HandEye 2（只需設定一次）

RealSense SDK、ROS 2 wrapper、UR 驅動程式及 MoveIt 的安裝方式，請參考 [ROS 2 建置指南](../ur3_ros2_setup.md)。以下步驟會將本資料夾的整合腳本加入 `easy_handeye2`，並註冊 `ros2 run` 指令。

### 2.1 取得 Easy HandEye 2 原始碼

若套件尚未放入工作區：

```bash
cd ~/ur3_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
```

若 `~/ur3_ws/src/easy_handeye2` 已存在，請使用現有的原始碼目錄。

### 2.2 複製整合校正程式

```bash
cp "$HOME/Desktop/robotic control/handeye/integrated_handeye_calibration.py" \
  "$HOME/ur3_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/integrated_handeye_calibration.py"
```

將檔案放在含有 `handeye_server.py` 與 `handeye_publisher.py` 的 Python 套件目錄。

### 2.3 在 `setup.py` 註冊指令

編輯 `~/ur3_ws/src/easy_handeye2/easy_handeye2/setup.py`，在 `entry_points` → `console_scripts` 清單加入：

```python
'integrated_auto_handeye_calibration = easy_handeye2.integrated_handeye_calibration:main',
```

### 2.4 確認 `package.xml` 執行相依項目

編輯 `~/ur3_ws/src/easy_handeye2/easy_handeye2/package.xml`，確認下列相依項目存在；只加入缺少的項目：

```xml
<exec_depend>aruco_opencv</exec_depend>
<exec_depend>moveit_py</exec_depend>
<exec_depend>moveit_configs_utils</exec_depend>
```

保留套件既有的 `rclpy`、`tf2_ros`、`geometry_msgs` 與 `easy_handeye2_msgs` 相依項目。

### 2.5 安裝相依套件、建置並確認指令

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select easy_handeye2 --symlink-install
source ~/ur3_ws/install/setup.bash
ros2 pkg executables easy_handeye2
```

執行檔清單中應包含：

```text
easy_handeye2 integrated_auto_handeye_calibration
```

## 3. 校正前啟動機器人與相機

依照 [ROS 2 建置指南第 6 節](../ur3_ros2_setup.md#6-launch-commands) 啟動 UR3 驅動程式、MoveIt/RViz 與 RealSense 相機，並保持執行。整合程式需要機器人提供 `/robot_description`、關節狀態與 TF，讓 MoveIt 規劃 15 個取樣姿態。

確認 UR 示教器已準備好接受 External Control、手臂周圍安全，且相機能看見 ID 582 標定板。預設影像 topic 為 `/camera/camera/color/image_raw`。可在 RViz2 新增 **Image** 顯示，或執行 `ros2 run rqt_image_view rqt_image_view` 檢查影像。

`marker_size` 指 ArUco 黑色方框的實際邊長，不包含白色外框；預設值為 `0.04` 公尺。

## 4. 執行整合校正

開啟新終端並執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run easy_handeye2 integrated_auto_handeye_calibration
```

程式會依序進行：

1. 啟動 ArUco tracker 並等待指定標記的 TF。尚未偵測到標記時，終端每隔數秒回報一次；等待標記沒有逾時限制。
2. 偵測到標記後詢問是否開始校正。輸入 `n` 會停止 tracker 並結束；輸入 `y` 才會啟動 Easy HandEye 2 校正介面。
3. 等待校正服務、機器人 TF 與標記 TF 就緒，再從手臂目前姿態開始自動取樣。程式不會先移動到 `test_configuration`。
4. 校正成功並儲存後，關閉 tracker 與校正介面，再詢問是否發布 TF。輸入 `y` 會持續發布，按 Ctrl+C 停止；輸入 `n` 則結束。

開始前請確認標記清楚可見、移動路徑安全，且 MoveIt 碰撞場景設定正確。校正會實際移動手臂，請確保急停裝置可立即使用。

### 可設定參數

預設校正類型是 `eye_on_base`，名稱為 `ur3_realsense_eob`。若要選擇 eye-in-hand：

```bash
ros2 run easy_handeye2 integrated_auto_handeye_calibration \
  --calibration-type eye_in_hand \
  --name ur3_realsense_eih
```

執行 `ros2 run easy_handeye2 integrated_auto_handeye_calibration --help` 可查看其他座標系、影像 topic、標記大小及啟動逾時參數。名稱用於儲存和發布結果，兩者必須一致。

## 5. 自動取樣方式

使用者確認開始後，程式會讀取當下 `base_link -> tool0` TF 作為起始姿態，並產生 15 個目標：

| 樣本 | 目標姿態 |
|---|---|
| 1–12 | 保持起始位置，分別繞工具座標系的 X、Y、Z 軸旋轉 `+20°`、`-20°`、`+10°`、`-10°`，共 12 個姿態。 |
| 13–15 | 保持起始方向，分別沿 `base_link` 的 X、Y、Z 軸平移 `+0.04 m`（4 公分）。 |

MoveIt 會規劃並執行每個目標。到達後程式等待 1.5 秒，再向 Easy HandEye 2 取得樣本。單一目標若規劃失敗，最多重試三次；若仍失敗，程式會透過 MoveIt IK 與 state-validity 服務診斷目標是否可解及是否碰撞，輸出起始與目標姿態及診斷結果，然後停止，不會執行該目標。

若開始前已有舊樣本，程式會中止，避免混入不同批次。每次取樣後，程式會確認樣本數有正確增加。規劃、取樣或樣本數檢查失敗時都會停止。若中途停止，已取得的部分樣本不會自動清除；重新執行前請先在 Easy HandEye 2 介面清除舊樣本。15 個樣本完成後，手臂會返回起始姿態，再計算並儲存校正結果。

請確保整個流程中標記都清楚可見，並確認目標位置與路徑安全。若流程中止，手臂可能停在當前姿態；繼續操作前請檢查機器人狀態。

## 6. 校正結果

校正檔預設儲存在：

```text
~/.ros2/easy_handeye2/calibrations/<name>.calib
```

終端會輸出校正類型、TF parent/child 座標系、平移、四元數及 4×4 矩陣。若在發布提示選擇 `y`，整合流程會以相同的 `<name>` 啟動 `publish.launch.py`，持續發布校正 TF。

## 參考資料

- UR3 ROS 2 驅動程式：[Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Easy HandEye 2 說明文件](https://github.com/marcoesposito1988/easy_handeye2)
- [RealSense ROS 2 Wrapper](https://github.com/realsenseai/realsense-ros)
- [aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv)
