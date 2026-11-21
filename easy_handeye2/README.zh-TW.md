# Easy HandEye 2 校正服務與手動介面

[English](README.md)

`easy_handeye2` 提供通用的手眼採樣、OpenCV 校正計算、結果儲存，以及 rqt 校正／評估介面。它不產生 UR3 移動路徑，也不啟動 ArUco。自動流程與相機根座標 TF 發布請使用 [auto_calibration](../auto_calibration/README.zh-TW.md)。

此處是本專案維護的 Easy HandEye 2 核心，保留既有 TF／服務錯誤處理與重複 server 防護，並非完全未修改的官方來源。UR 專用移動與 publisher 整合屬於 `auto_calibration`；核心不依賴 MoveItPy、UR 設定或 ArUco。

## 安裝

完成[主機建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節一次即可。此處已提供核心來源；依講義只從官方取得 `easy_handeye2_msgs`，不要在工作區另放第二份 Easy HandEye 主程式。

`package.xml` 宣告 rclpy、TF、ROS 介面、OpenCV、NumPy、transforms3d、YAML 與 Qt／rqt 相依，由共用 rosdep 指令安裝。核心不需要 Docker、AnyGrasp 或 MoveIt；自動流程另有 `auto_calibration` 的相依。

## 手動校正

預設為固定式相機，請保持相機不動，將 ArUco 標記牢固固定於手臂工具。先啟動 [bringup](../ur3_bringup/README.zh-TW.md)，再於獨立終端啟動 tracker：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run aruco_opencv aruco_tracker_autostart --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_size:=0.04 -p marker_dict:=ARUCO_ORIGINAL
```

Marker size 必須是實際標記邊長，單位公尺。另開終端啟動手動校正 GUI 與服務：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch easy_handeye2 calibrate.launch.py \
  name:=ur3_realsense_eob calibration_type:=eye_on_base \
  robot_base_frame:=base_link robot_effector_frame:=tool0 \
  tracking_base_frame:=camera_color_optical_frame \
  tracking_marker_frame:=marker_582
```

透過既有手臂控制方式移動，在 GUI 中先清空樣本，再於不同角度／位置採樣、計算與儲存。此手動介面不會移動手臂，也不執行自動流程的預檢與樣本清零。請勿與自動校正同時執行，因為兩者共用一組服務命名空間。

| Launch 參數 | 意義 |
| --- | --- |
| `name` | 校正檔名稱。 |
| `calibration_type` | `eye_on_base`：固定相機、標記在工具；`eye_in_hand`：相機在工具、標記固定。 |
| `robot_base_frame` | 手臂參考座標。 |
| `robot_effector_frame` | 手臂末端座標。 |
| `tracking_base_frame` | Tracker 回報標記姿態所用的座標。 |
| `tracking_marker_frame` | 標記座標名稱，需符合實際 ID。 |

六個參數皆須提供。校正服務分別讀取 robot-base→tool 與 camera→marker 兩條 TF，不會用假轉換把兩棵樹接起來。

## 儲存結果與 TF

校正結果位於 `~/.ros2/easy_handeye2/calibrations/<name>.calib`，樣本位於 `~/.ros2/easy_handeye2/samples/`。核心儲存校正 tracking frame 的姿態。RealSense 請使用 [auto_calibration 的 publisher](../auto_calibration/README.zh-TW.md)掛接相機根座標，保留相機內部 TF：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py \
  name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

同時只啟動一個校正 publisher；重新載入、單一發布者及相機就緒條件見自動校正講義。

## 評估介面

儲存校正並備妥所需 TF／tracker 後，可啟動評估 GUI：

```bash
ros2 launch easy_handeye2 evaluate.launch.py name:=ur3_realsense_eob
```

`rqt_calibrator.py` 與 `rqt_evaluator.py` 是已註冊的 console entry point；GUI 資源與 plugin 描述仍由本套件提供。

## 服務行為

服務命名空間為 `/easy_handeye2/calibration/`，包含 `take_sample`、`get_sample_list`、`get_current_transforms`、`remove_sample`、`save_samples`、`load_samples`、`compute_calibration`、`save_calibration`、`list_algorithms` 與 `set_algorithm`，介面由 `easy_handeye2_msgs` 提供。

TF 不存在或樹斷開時，查詢不回傳有效姿態、採樣不新增資料。載入不存在或無效樣本檔時回傳 `success=false` 並保留原樣本；刪除最後樣本可接受空清單。計算錯誤回傳 `valid=false`。本機同一 ROS domain 限制一個 server 持有這組服務；這些防護獨立於自動移動流程。

套件採用隨附 LGPLv3 授權，保留官方作者資訊。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [LICENSE.md](LICENSE.md) | 隨附 LGPLv3 授權條款與原始授權文字。 |
| [README.md](README.md) | 英文操作指南：通用校正服務、手動校正／評估 GUI 與儲存結果的使用方式。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：通用校正服務、手動校正／評估 GUI 與儲存結果的使用方式。 |
| [easy_handeye2/__init__.py](easy_handeye2/__init__.py) | 定義樣本／校正目錄、服務命名空間與名稱，提供本機資源互斥鎖。 |
| [easy_handeye2/handeye_calibration.py](easy_handeye2/handeye_calibration.py) | 校正參數宣告與讀取、校正檔路徑，以及 YAML 校正檔載入／儲存。 |
| [easy_handeye2/handeye_calibration_backend_opencv.py](easy_handeye2/handeye_calibration_backend_opencv.py) | 將 ROS 樣本轉換為 OpenCV 格式，呼叫手眼演算法並產生校正結果。 |
| [easy_handeye2/handeye_client.py](easy_handeye2/handeye_client.py) | 供手動 GUI 使用的採樣、演算法選擇、校正計算與儲存服務 client。 |
| [easy_handeye2/handeye_rqt_calibrator.py](easy_handeye2/handeye_rqt_calibrator.py) | 註冊手動校正 rqt plugin，提供 rqt_calibrator.py 執行入口及介面生命週期。 |
| [easy_handeye2/handeye_rqt_calibrator_widget.py](easy_handeye2/handeye_rqt_calibrator_widget.py) | 載入手動校正 UI、更新姿態／樣本顯示，以及處理採樣、計算與儲存按鈕。 |
| [easy_handeye2/handeye_rqt_evaluator.py](easy_handeye2/handeye_rqt_evaluator.py) | 註冊校正評估 rqt plugin，提供 rqt_evaluator.py 執行入口及介面生命週期。 |
| [easy_handeye2/handeye_rqt_evaluator_widget.py](easy_handeye2/handeye_rqt_evaluator_widget.py) | 載入評估 UI，讀取校正與目前 TF，顯示姿態及一致性評估資訊。 |
| [easy_handeye2/handeye_sampler.py](easy_handeye2/handeye_sampler.py) | 讀取手臂與標記 TF、建立／刪除樣本，以及載入和儲存樣本清單。 |
| [easy_handeye2/handeye_server.py](easy_handeye2/handeye_server.py) | 提供採樣／樣本管理、演算法選擇、計算與儲存服務，處理錯誤及重複 server。 |
| [launch/calibrate.launch.py](launch/calibrate.launch.py) | 啟動手動校正 server 與 rqt GUI，接收校正名稱、類型與四個 frame 參數。 |
| [launch/evaluate.launch.py](launch/evaluate.launch.py) | 依校正名稱啟動 rqt 評估介面。 |
| [package.xml](package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [plugin_calibrator.xml](plugin_calibrator.xml) | 向 rqt 宣告手動校正 plugin 類別與選單資訊。 |
| [plugin_evaluator.xml](plugin_evaluator.xml) | 向 rqt 宣告校正評估 plugin 類別與選單資訊。 |
| [resource/easy_handeye2](resource/easy_handeye2) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [resource/rqt_handeye.ui](resource/rqt_handeye.ui) | Qt Designer 手動校正畫面，包含樣本清單與採樣／計算／儲存控制。 |
| [resource/rqt_handeye_evaluator.ui](resource/rqt_handeye_evaluator.ui) | Qt Designer 校正評估畫面。 |
| [resource/rqt_handeye_info.ui](resource/rqt_handeye_info.ui) | 校正與評估介面共用的校正參數／frame 資訊畫面。 |
| [setup.cfg](setup.cfg) | 將 easy_handeye2 的註冊執行檔安裝到 lib/easy_handeye2，供 ros2 run 尋找。 |
| [setup.py](setup.py) | 註冊校正 server 與兩個 rqt 指令，安裝 launch、GUI／plugin 資源、授權與雙語 README。 |
