# UR3 + RealSense：Easy HandEye 2 手眼標定

## 硬體規格

| 項目 | 設定 |
|---|---|
| 作業系統 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| 機械手臂 | Universal Robots UR3 |
| UR3 IP | `192.168.86.7` |
| 工作區 | `~/ur3_ws` |
| 相機 | RealSense（ROS 2 wrapper）|
| 標定標記 | ArUco `ARUCO_ORIGINAL`，ID 582，邊長 40 mm |
| 標定軟體 | `easy_handeye2` |

依序完成相機安裝、環境啟動與標定流程。

## 1. 決定校正類型

依相機與標定板的固定方式選擇：

| 相機 | ArUco 板 | `calibration_type` |
|---|---|---|
| 固定在桌上/支架 | 固定在 UR3 末端工具 | `eye_on_base` |
| 固定在 UR3 末端工具 | 固定在工作區 | `eye_in_hand` |

**校正類型是在第 6 節啟動 Easy HandEye 2 的指令中設定**：相機固定選 `calibration_type:=eye_on_base`；相機裝在末端選 `calibration_type:=eye_in_hand`。

## 2. 安裝 Easy HandEye 2 與加入自動校正程式

RealSense SDK 與 ROS 2 wrapper 的安裝請依 [ROS2 setup 講義](../ur3_ros2_setup.md) 操作。本節安裝 Easy HandEye 2，並將自行設計的 `auto_handeye_calibration.py` 加入它的 Python 套件，安裝後就能使用 `ros2 run easy_handeye2 auto_handeye_calibration`。

### 2.1 取得 Easy HandEye 2 原始碼

如果尚未下載套件，執行：

```bash
cd ~/ur3_ws/src
git clone https://github.com/marcoesposito1988/easy_handeye2.git
```

若 `~/ur3_ws/src/easy_handeye2` 已存在，略過 clone，直接在該資料夾加入程式。

### 2.2 將自動校正程式放進套件

把本講義資料夾附帶的 `auto_handeye_calibration.py` 複製到 Easy HandEye 2 的 Python module 目錄：

```bash
cp "$HOME/Desktop/robotic control/handeye/auto_handeye_calibration.py" \
  "$HOME/ur3_ws/src/easy_handeye2/easy_handeye2/easy_handeye2/auto_handeye_calibration.py"
```

此範例假設講義資料夾位於桌面上的 `handeye/`；若放在其他位置，請將來源路徑改成實際位置。程式必須和 `handeye_server.py`、`handeye_publisher.py` 放在同一層 `easy_handeye2/` Python 套件目錄。

### 2.3 註冊 `ros2 run` 指令

開啟 `~/ur3_ws/src/easy_handeye2/easy_handeye2/setup.py`，在 `entry_points` → `console_scripts` 清單加入一行：

```python
'auto_handeye_calibration = easy_handeye2.auto_handeye_calibration:main',
```

例如放在 `handeye_calibration_commander` 那一行後面，注意前一行結尾要有逗號。

### 2.4 加入執行相依套件

開啟 `~/ur3_ws/src/easy_handeye2/easy_handeye2/package.xml`，在其他 `<exec_depend>` 項目旁加入：

```xml
<exec_depend>moveit_py</exec_depend>
<exec_depend>moveit_configs_utils</exec_depend>
```

Easy HandEye 2 原本已有 `easy_handeye2_msgs`、`geometry_msgs`、`rclpy` 和 `tf2_ros` 相依項目，不需重複加入。

### 2.5 完成建置並確認安裝

完成 Easy HandEye 2 原始碼與自動校正程式整合後，回到 [ROS2 setup 講義第 4 節](../ur3_ros2_setup.md#4-原始碼下載與編譯)，依照其中的指令安裝相依套件並執行 `colcon build --symlink-install`。建置完成後載入 workspace，再確認程式已註冊：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 pkg executables easy_handeye2
```

清單中應有 `easy_handeye2 auto_handeye_calibration`。RealSense 相機的啟動方式請參考 setup 講義第 6 節。

## 3. 啟動 ROS 2 環境

UR3 driver、MoveIt/RViz 與 RealSense 的啟動方式請直接參考 [ROS2 setup 講義第 6 節：啟動指令](../ur3_ros2_setup.md#6-啟動指令)。實體標定前，請確認 UR 示教器已啟用 External Control、相機已連接 USB 3.x，並讓相關終端保持執行。

## 4. 啟動 ArUco 偵測

在終端機 4 啟動 tracker：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_size:=0.04 \
  -p marker_dict:=ARUCO_ORIGINAL
```

把 ID 582 標定板放在相機視野內。tracker 會辨識字典中的 marker 並發布 TF；不用把 ID 另作啟動參數。

## 5. 找出要填入的 TF frame 名稱

Easy HandEye 2 需要 UR 基座、UR 末端工具、相機 optical frame 和 marker frame 的名稱。執行一次：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run tf2_tools view_frames
```

在產生的 `frames.pdf` 找到這四個 frame。UR 常見名稱是 `base_link`、`tool0`；RealSense 常見名稱是 `camera_color_optical_frame` ;marker frame 以實體IP填入`marker_<marker ID>`。

## 6. 啟動 Easy HandEye 2 校正介面

在終端機 5 載入環境，並使用上一步查到的 frame 名稱。只選一種執行。

### 相機固定、標定板裝在末端：`eye_on_base`

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=ur3_realsense_eob \
  robot_base_frame:=base_link \
  robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=marker_582
```

## 7. 在 RViz2 顯示相機影像（建議先確認）

影像 topic 已確認為 `/camera/camera/color/image_raw`。如果 setup 講義的啟動流程已開啟 RViz2，可直接在該 RViz2 加入影像；否則另開終端機啟動：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
rviz2
```

在 RViz2 左下角按 **Add** → **By display type** → **Image** → **OK**。選取新增的 **Image** display，在左側屬性將 **Topic** 設為：

```text
/camera/camera/color/image_raw
```

若清單沒有該 topic，先檢查相機是否有發布影像：

```bash
ros2 topic list | grep image
ros2 topic hz /camera/camera/color/image_raw
```

也可用 `rqt_image_view` 單獨檢查影像：

```bash
ros2 run rqt_image_view rqt_image_view
```

在視窗的 topic 選單選 `/camera/camera/color/image_raw`。影像應能看到清楚、完整的 ArUco 標記。

## 8. 自動移動與手眼校正

Easy HandEye 2 目前的自動移動服務尚未完成。本 workspace 提供 `easy_handeye2 auto_handeye_calibration` 指令，以 MoveItPy 移動約 15 個姿態，再呼叫 Easy HandEye 2 原生 service 取樣、計算及儲存結果。

若剛安裝或更新 Easy HandEye 2，請先完成本講義第 2.5 節的相依安裝與編譯。啟動前確認 UR driver、MoveIt/RViz、RealSense、ArUco tracker 與第 6 節的 Easy HandEye 校正介面都在執行；先檢查 marker 可見、TF 正常，並清空校正介面中的舊樣本。

在新的終端機執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run easy_handeye2 auto_handeye_calibration
```

程式會先檢查是否有舊樣本，若有便中止以免混入不同批次資料。確認 marker 全程可見、手臂路徑與 MoveIt 碰撞場景安全後，輸入 `START` 開始自動移動。移動或取樣失敗時流程會停止；手臂可能停在當前位置，請檢查機器人狀態。

完成後 terminal 會輸出 TF 的 parent/child frame、平移、四元數和 4×4 矩陣，並將結果存入 `~/.ros2/easy_handeye2/calibrations/<name>.calib`。確認輸出後，依第 10 節發布校正。此程式會實際驅動手臂；操作時請保持急停可用。

## 9. 手動取樣、計算、保存

Easy HandEye 2 的校正視窗通常是 RQt GUI。確認視窗已開啟，且 robot 與 tracking 的 TF 都能更新，再依下列步驟操作：

1. 在 RViz2 的 Image 視窗確認 marker 清楚、完整且沒有被遮住；手臂和相機/標定板的安裝也都要固定牢靠。
2. 若使用上方自動程式，依提示執行並等待 terminal 顯示計算結果與儲存路徑。若使用手動方式，透過示教器或 Free Drive 移到約 15–25 個姿態，在每個穩定且 marker 可見的姿態按 **Take Sample**。
3. 手動收樣完成後按 **Compute**。確認介面顯示計算結果/變換值且沒有錯誤，再按 **Save**。記下介面回報的檔案位置和校正名稱。

固定好相機和標定板，避免反光、遮擋與模糊。`marker_size` 必須是黑色方框實際邊長，不含白色外框。

Easy HandEye 2 預設將校正檔存到：

```text
~/.ros2/easy_handeye2/calibrations/<name>.calib
```

以介面或終端機回報的實際保存路徑為準。

## 10. 發布校正結果

計算並保存後，結束校正 launch（`Ctrl+C`），保留 UR3、RealSense 和 ArUco tracker 運作。在新終端機發布結果；`name` 要與校正時相同：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch easy_handeye2 publish.launch.py name:=ur3_realsense_eob
```

若使用 `eye_in_hand`，將最後一行的名稱換成 `ur3_realsense_eih`。發布後可在 RViz2 加入 **TF** display 查看座標關係；Image display 則可繼續用來確認相機影像與 marker 偵測狀況。

## 參考

- UR3 ROS 2 driver：[Universal Robots ROS 2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [Easy HandEye 2 官方說明](https://github.com/marcoesposito1988/easy_handeye2)
- [RealSense ROS 2 Wrapper](https://github.com/realsenseai/realsense-ros)
- [aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv)
