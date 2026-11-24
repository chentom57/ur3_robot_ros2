# UR3 視覺夾取專案

[English](readme.md)

本專案整合 UR3 機械手臂、Robotiq 2F-85 夾爪、RealSense 深度相機、ROS 2、MoveIt 與 AnyGrasp，提供從環境建置、手眼校正，到點雲夾取姿態推論與手臂執行的程式及操作講義。

主機負責 ROS 通訊、相機資料、座標轉換、路徑規劃與硬體控制；AnyGrasp 在獨立的 GPU Docker 容器內執行。本資料夾提供可分享的原始碼與文件，實際建置及執行使用自己的 ROS 工作區。

## 專案目標

本專案希望完成「物體辨識後夾取」：從相機畫面辨識目標物體，取得其位置或分割範圍，再估算夾取姿態，經過座標轉換與路徑規劃，讓手臂夾取該物體。

目前已具備即時點雲擷取、AnyGrasp 候選姿態生成、座標轉換、規劃與確認後執行。物體辨識，以及辨識目標與夾取候選之間的對應仍待整合；現階段依人工圈選區域、夾爪開口、評分與規劃可行性選擇姿態。

## 專案功能

| 功能 | 說明 |
| --- | --- |
| 系統啟動 | 整合手臂 driver、MoveIt、RViz、虛擬桌面與選用的 RealSense 相機，支援實體或 mock hardware。 |
| 手眼校正 | 自動移動採樣、計算及儲存校正，發布或重新載入手臂與相機之間的 TF。預設是相機固定的 eye-on-base 配置。 |
| 即時姿態推論 | 擷取新的相機點雲，將人工圈選區域遮罩透過檔案傳入 Docker，取得 AnyGrasp 候選姿態。 |
| 候選規劃 | 按分數由高到低嘗試有效姿態，選擇第一個去程與返回記錄起始位置都能規劃成功的候選。 |
| 夾取執行 | 執行前要求確認；先開爪，再移至候選姿態並閉爪，接著返回記錄的起始位置，等候使用者確認放爪。 |
| RViz 顯示 | 顯示 TCP 目標、姿態三軸、去回程路徑及桌面碰撞標記。 |
| 僅規劃 | `plan_only:=true` 仍會擷取點雲、執行推論及規劃，但不下達手臂或夾爪命令。 |

每次啟動 `pick_up` 擷取一個場景，最多執行一個候選；目前沒有連續循環夾取或依物體名稱選取目標的介面。AnyGrasp 根據幾何產生可夾取姿態，沒有整合 SAM 或語意物體辨識。

## 資料夾與工具

| 資料夾 | 功能 | 使用工具 | 操作文件 |
| --- | --- | --- | --- |
| `ROS2+driver/` | 建立主機 ROS、手臂與相機環境的基礎講義；不包含 driver 原始碼。 | Ubuntu、ROS 2 Jazzy、UR ROS 2 Driver、RealSense SDK／ROS driver、rosdep、colcon、網路診斷工具 | [操作講義](ROS2+driver/ur3_ros2_setup.zh-TW.md) |
| `ur3_moveit_scene/` | 共用虛擬碰撞桌面。 | Python、rclpy、MoveIt Planning Scene、RViz Marker／MarkerArray | [操作講義](ur3_moveit_scene/README.zh-TW.md) |
| `ur3_bringup/` | 系統整合啟動及實體／虛擬模式設定，附本專案的 MoveIt／RViz 整合。 | ROS 2 launch、UR driver、MoveIt、RViz、RealSense、CMake | [操作講義](ur3_bringup/README.zh-TW.md) |
| `easy_handeye2/` | 通用採樣服務、OpenCV 校正計算、儲存與手動／評估 GUI，保留必要修補。 | Python、TF2、OpenCV、Qt／rqt、NumPy、transforms3d | [操作講義](easy_handeye2/README.zh-TW.md) |
| `auto_calibration/` | UR3 自動採樣流程與 TF 發布管理，透過服務使用 easy_handeye2。 | MoveItPy、ArUco、TF2、ROS 服務與子程序管理 | [操作講義](auto_calibration/README.zh-TW.md) |
| `docker+anygrasp/` | Docker 建置檔、GPU 與 SDK 安裝／授權講義，以及官方範例的單次推論程式。 | Docker Engine、NVIDIA Container Toolkit、CUDA、PyTorch、MinkowskiEngine、PointNet2、GraspNetAPI、AnyGrasp SDK | [操作講義](docker+anygrasp/README.zh-TW.md) |
| `ur3_pick_up/` | 即時夾取主程式：相機接收、點雲處理、Docker 橋接、候選轉換、規劃及夾爪控制。 | Python、rclpy、PointCloud2、TF2、NumPy、MoveItPy、Docker CLI、Robotiq TCP socket | [操作講義](ur3_pick_up/README.zh-TW.md) |

`ur3_pick_up/scripts/anygrasp_worker.py` 會被傳入 Docker 執行；其他 ROS 模組在主機執行。Docker 不直接訂閱 ROS 相機 topic。官方 `demo_once.py` 使用的是 SDK 範例場景，不是目前相機看到的場景。

## 環境與資料位置

| 項目 | 目前設定 |
| --- | --- |
| 主機 | Ubuntu 24.04、ROS 2 Jazzy、Python 3.12 |
| 硬體 | UR3、Robotiq 2F-85、RealSense D435；固定式相機 |
| 模型環境 | Docker 內 Python 3.11、PyTorch 2.5.1 + CUDA 12.1、MinkowskiEngine 0.5.4 |
| GPU 建置設定 | RTX 3060 Laptop；Dockerfile 的 CUDA 架構為 `8.6`。其他 GPU 需調整。 |
| ROS 工作區 | `~/ur3_ws`：安裝來源、建置結果及 ROS 執行環境 |
| 模型工作目錄 | `~/anygrasp_docker`：Docker 建置、點雲交換與推論紀錄 |
| 預設容器 | `anygrasp-fixed` |
| 校正檔 | `~/.ros2/easy_handeye2/calibrations/` |
| 規劃群組 | `ur_manipulator`；預設返回命名姿態：`home` |
| 相機座標 | `camera_color_optical_frame` |
| 點雲 topic | `/camera/camera/depth/color/points` |
| 夾取中心 | 預設為 `tool0` 的 +Z 方向 0.174 m，軸向對應見夾爪設定檔。 |

硬體位址、TCP 與相機位置需依實際安裝設定。AnyGrasp SDK、授權與模型權重依 Docker 講義另行取得，本專案不附這些檔案。

## 相依套件準備

整份專案可放在 `~/ur3_ws/src/robot_arm`，資料夾名稱可自訂。`colcon` 會遞迴找到五個 ROS 套件：`easy_handeye2`、`auto_calibration`、`ur3_bringup`、`ur3_moveit_scene` 與 `ur3_pick_up`。每個套件名稱只能有一份，不要再把同一份子資料夾另外複製到 `src`。

| 相依項目 | 安裝位置與步驟 |
| --- | --- |
| ROS 2 Jazzy、rosdep、colcon 與主機 Python 函式庫 | [主機建置講義](ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節；Python 相依由第 4 節 rosdep 安裝。 |
| 官方 UR driver 與 `ur_moveit_config` | [主機講義](ROS2+driver/ur3_ros2_setup.zh-TW.md)第 4 節的 GitHub `jazzy` 來源；UR description／client library 由 rosdep 安裝。 |
| RealSense ROS 4.59.1 與系統 SDK | 依主機講義下載相機來源；SDK 需 2.59.0 以上，目前使用 2.59.1。rosdep 加 `--skip-keys librealsense2`，避免另一套 SDK。 |
| `easy_handeye2_msgs` 0.5.0 | [主機講義第 4 節](ROS2+driver/ur3_ros2_setup.zh-TW.md#handeye-message-dependency)的固定 revision、sparse checkout；不取出官方主程式，避免重複套件。 |
| ArUco、MoveItPy、Qt／rqt、OpenCV、NumPy、transforms3d 與 YAML | 由[主機講義第 4 節](ROS2+driver/ur3_ros2_setup.zh-TW.md)的 rosdep 指令依 manifest 一次安裝。 |
| GPU Docker、AnyGrasp SDK、授權與 detection 權重 | [Docker 講義](docker+anygrasp/README.zh-TW.md)第 3～7 節；夾取推論需要，校正不需要。 |

先備妥來源，再執行 rosdep。首次使用以 `sudo rosdep init` 初始化，接著 `rosdep update`，再依主機講義於 `~/ur3_ws` 安裝相依與建置。rosdep 若有錯誤，先解決再編譯。

建議來源結構：

```text
~/ur3_ws/src/
├── robot_arm/                       # 本專案完整資料夾
│   ├── easy_handeye2/                # 通用校正服務與手動介面
│   ├── auto_calibration/             # UR3 自動流程與 TF 發布
│   ├── ur3_bringup/
│   ├── ur3_moveit_scene/
│   └── ur3_pick_up/
├── Universal_Robots_ROS2_Driver/     # 官方 GitHub 來源
├── realsense-ros/                   # 官方來源，tag 4.59.1
└── easy_handeye2_interfaces/
    └── easy_handeye2_msgs/          # 只保留訊息套件
```

既有工作區若已分開放置套件，可沿用原結構。新增來源前先用 `colcon list` 確認套件名稱，避免重複。

## 從零建構的閱讀順序

**第一次使用請先讀 [ROS2+driver 的環境建置講義](ROS2+driver/ur3_ros2_setup.zh-TW.md)，不要直接從 pickup 的啟動指令開始。** 該資料夾的入口檔名稱是 `ur3_ros2_setup.zh-TW.md`／`ur3_ros2_setup.md`，不是 `README.md`。

| 順序 | 優先閱讀 | 完成目標 |
| --- | --- | --- |
| 1 | `ROS2+driver/`：[操作講義](ROS2+driver/ur3_ros2_setup.zh-TW.md) | 安裝 ROS、開發工具、手臂與相機 driver，建立 `ur3_ws`，確認網路。 |
| 2 | `ur3_moveit_scene/`：[操作講義](ur3_moveit_scene/README.zh-TW.md) | 了解已建置的場景套件、桌面高度與碰撞設定。 |
| 3 | `ur3_bringup/`：[操作講義](ur3_bringup/README.zh-TW.md) | 啟動系統，確認手臂狀態、RViz、相機與點雲。 |
| 4 | `auto_calibration/`：[操作講義](auto_calibration/README.zh-TW.md) | 確認校正相依就緒、固定標記並完成採樣，儲存及發布相機與手臂的 TF。 |
| 5 | `docker+anygrasp/`：[操作講義](docker+anygrasp/README.zh-TW.md) | 安裝 GPU Docker 環境、建置 SDK、取得授權與權重，先完成官方範例推論。此步可獨立準備。 |
| 6 | `ur3_pick_up/`：[操作講義](ur3_pick_up/README.zh-TW.md) | 設定夾爪 TCP；先以 `plan_only:=true` 驗證即時點雲、推論及規劃，再操作實體夾取。 |

手動校正或查閱通用服務時，另讀 [easy_handeye2 講義](easy_handeye2/README.zh-TW.md)；日常自動校正使用 `auto_calibration`，不必手動啟動 easy_handeye2 GUI。

主機講義第 1～4 節只需完成一次，後續套件 README 直接閱讀功能、操作與參數。官方 UR driver、RealSense ROS 與 `easy_handeye2_msgs` 來源先下載到工作區；ROS／Python 相依交由 rosdep 安裝，最後 colcon 建置。系統 RealSense SDK、GPU Docker 與 AnyGrasp 授權／權重有各自的必要準備步驟；僅複製本專案不會自動建立完整環境。

## 不同設備條件可驗證的範圍

| 設備 | 可執行內容 |
| --- | --- |
| 無手臂、無相機 | Mock bringup、RViz 場景與虛擬桌面；有 GPU 容器時可另跑官方範例推論。 |
| 有相機、無手臂 | `use_mock_hardware:=true launch_camera:=true`，確認點雲，再配合校正 TF 與 `plan_only:=true` 排查接收、推論及虛擬規劃。相機若移位，舊校正不能驗證實際位置精度。 |
| 手臂與相機皆有 | 完成手眼校正後，執行即時規劃與確認後的實體夾取。 |

`plan_only:=true` 不會產生模擬點雲，仍需要相機資料、TF、MoveIt 場景及可用的 AnyGrasp 容器。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表列出本專案的原始檔，路徑相對於專案根目錄；不含 .git 內容、建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [.gitignore](.gitignore) | Git 忽略規則，排除建置結果、快取與本機產物。 |
| [AGENTS.md](AGENTS.md) | 專案協作指引；指定 PowerPoint 最終檔案的存放位置。 |
| [ROS2+driver/ur3_ros2_setup.md](ROS2+driver/ur3_ros2_setup.md) | 英文從零建置指南：ROS、官方來源、系統 SDK、rosdep、建置與連線檢查。 |
| [ROS2+driver/ur3_ros2_setup.zh-TW.md](ROS2+driver/ur3_ros2_setup.zh-TW.md) | 繁體中文從零建置指南：ROS、官方來源、系統 SDK、rosdep、建置與連線檢查。 |
| [auto_calibration/LICENSE.md](auto_calibration/LICENSE.md) | 隨附 LGPLv3 授權條款與原始授權文字。 |
| [auto_calibration/README.md](auto_calibration/README.md) | 英文操作指南：自動校正、啟動清理、參數、TF 發布及操作檢查。 |
| [auto_calibration/README.zh-TW.md](auto_calibration/README.zh-TW.md) | 繁體中文操作指南：自動校正、啟動清理、參數、TF 發布及操作檢查。 |
| [auto_calibration/auto_calibration/__init__.py](auto_calibration/auto_calibration/__init__.py) | 標示自動校正 Python 套件並提供套件簡介。 |
| [auto_calibration/auto_calibration/calibrate.py](auto_calibration/auto_calibration/calibrate.py) | 自動校正入口；協調啟動清理、tracker、校正服務、確認提示、移動程序與 TF 發布。 |
| [auto_calibration/auto_calibration/motion.py](auto_calibration/auto_calibration/motion.py) | 服務 client、15 點姿態生成、規劃診斷與重試、自動移動、樣本清零及計算／儲存流程。 |
| [auto_calibration/auto_calibration/publisher.py](auto_calibration/auto_calibration/publisher.py) | 讀取校正檔，換算並發布相機根座標 TF，限制單一發布者並提供 reload 服務。 |
| [auto_calibration/auto_calibration/startup.py](auto_calibration/auto_calibration/startup.py) | 辨識並停止舊校正與同一相機 tracker 程序，檢查 ROS 衝突與資源鎖，阻擋未清理完成的啟動。 |
| [auto_calibration/launch/publish.launch.py](auto_calibration/launch/publish.launch.py) | 以校正名稱與相機根座標參數，啟動已儲存校正的 TF publisher。 |
| [auto_calibration/package.xml](auto_calibration/package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [auto_calibration/resource/auto_calibration](auto_calibration/resource/auto_calibration) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [auto_calibration/setup.cfg](auto_calibration/setup.cfg) | 將 auto_calibration 的註冊執行檔安裝到 lib/auto_calibration，供 ros2 run 尋找。 |
| [auto_calibration/setup.py](auto_calibration/setup.py) | 註冊 calibrate 與 handeye_publisher 指令，安裝 launch、授權與雙語 README。 |
| [docker+anygrasp/.dockerignore](docker+anygrasp/.dockerignore) | 排除授權、權重封存檔、結果與快取等不需送入 Docker build 的內容。 |
| [docker+anygrasp/.gitignore](docker+anygrasp/.gitignore) | 排除本機 SDK 素材、授權、權重封存檔、推論結果與快取，避免提交。 |
| [docker+anygrasp/Dockerfile](docker+anygrasp/Dockerfile) | 以 PyTorch／CUDA 開發映像建立基底，設定 GPU 架構並編譯官方指定分支的 MinkowskiEngine。 |
| [docker+anygrasp/Dockerfile.sdk](docker+anygrasp/Dockerfile.sdk) | 在 MinkowskiEngine 映像上安裝 SDK、PointNet2、GraspNetAPI、Open3D 相依與必要系統函式庫。 |
| [docker+anygrasp/README.md](docker+anygrasp/README.md) | 英文操作指南：Docker／GPU 安裝、SDK 建置、授權與權重準備、推論及容器操作。 |
| [docker+anygrasp/README.zh-TW.md](docker+anygrasp/README.zh-TW.md) | 繁體中文操作指南：Docker／GPU 安裝、SDK 建置、授權與權重準備、推論及容器操作。 |
| [docker+anygrasp/demo_once.py](docker+anygrasp/demo_once.py) | 只對 SDK 官方範例場景推論一次並儲存候選，供模型環境驗證，不使用即時相機。 |
| [easy_handeye2/LICENSE.md](easy_handeye2/LICENSE.md) | 隨附 LGPLv3 授權條款與原始授權文字。 |
| [easy_handeye2/README.md](easy_handeye2/README.md) | 英文操作指南：通用校正服務、手動校正／評估 GUI 與儲存結果的使用方式。 |
| [easy_handeye2/README.zh-TW.md](easy_handeye2/README.zh-TW.md) | 繁體中文操作指南：通用校正服務、手動校正／評估 GUI 與儲存結果的使用方式。 |
| [easy_handeye2/easy_handeye2/__init__.py](easy_handeye2/easy_handeye2/__init__.py) | 定義樣本／校正目錄、服務命名空間與名稱，提供本機資源互斥鎖。 |
| [easy_handeye2/easy_handeye2/handeye_calibration.py](easy_handeye2/easy_handeye2/handeye_calibration.py) | 校正參數宣告與讀取、校正檔路徑，以及 YAML 校正檔載入／儲存。 |
| [easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py](easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py) | 將 ROS 樣本轉換為 OpenCV 格式，呼叫手眼演算法並產生校正結果。 |
| [easy_handeye2/easy_handeye2/handeye_client.py](easy_handeye2/easy_handeye2/handeye_client.py) | 供手動 GUI 使用的採樣、演算法選擇、校正計算與儲存服務 client。 |
| [easy_handeye2/easy_handeye2/handeye_rqt_calibrator.py](easy_handeye2/easy_handeye2/handeye_rqt_calibrator.py) | 註冊手動校正 rqt plugin，提供 rqt_calibrator.py 執行入口及介面生命週期。 |
| [easy_handeye2/easy_handeye2/handeye_rqt_calibrator_widget.py](easy_handeye2/easy_handeye2/handeye_rqt_calibrator_widget.py) | 載入手動校正 UI、更新姿態／樣本顯示，以及處理採樣、計算與儲存按鈕。 |
| [easy_handeye2/easy_handeye2/handeye_rqt_evaluator.py](easy_handeye2/easy_handeye2/handeye_rqt_evaluator.py) | 註冊校正評估 rqt plugin，提供 rqt_evaluator.py 執行入口及介面生命週期。 |
| [easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py](easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py) | 載入評估 UI，讀取校正與目前 TF，顯示姿態及一致性評估資訊。 |
| [easy_handeye2/easy_handeye2/handeye_sampler.py](easy_handeye2/easy_handeye2/handeye_sampler.py) | 讀取手臂與標記 TF、建立／刪除樣本，以及載入和儲存樣本清單。 |
| [easy_handeye2/easy_handeye2/handeye_server.py](easy_handeye2/easy_handeye2/handeye_server.py) | 提供採樣／樣本管理、演算法選擇、計算與儲存服務，處理錯誤及重複 server。 |
| [easy_handeye2/launch/calibrate.launch.py](easy_handeye2/launch/calibrate.launch.py) | 啟動手動校正 server 與 rqt GUI，接收校正名稱、類型與四個 frame 參數。 |
| [easy_handeye2/launch/evaluate.launch.py](easy_handeye2/launch/evaluate.launch.py) | 依校正名稱啟動 rqt 評估介面。 |
| [easy_handeye2/package.xml](easy_handeye2/package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [easy_handeye2/plugin_calibrator.xml](easy_handeye2/plugin_calibrator.xml) | 向 rqt 宣告手動校正 plugin 類別與選單資訊。 |
| [easy_handeye2/plugin_evaluator.xml](easy_handeye2/plugin_evaluator.xml) | 向 rqt 宣告校正評估 plugin 類別與選單資訊。 |
| [easy_handeye2/resource/easy_handeye2](easy_handeye2/resource/easy_handeye2) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [easy_handeye2/resource/rqt_handeye.ui](easy_handeye2/resource/rqt_handeye.ui) | Qt Designer 手動校正畫面，包含樣本清單與採樣／計算／儲存控制。 |
| [easy_handeye2/resource/rqt_handeye_evaluator.ui](easy_handeye2/resource/rqt_handeye_evaluator.ui) | Qt Designer 校正評估畫面。 |
| [easy_handeye2/resource/rqt_handeye_info.ui](easy_handeye2/resource/rqt_handeye_info.ui) | 校正與評估介面共用的校正參數／frame 資訊畫面。 |
| [easy_handeye2/setup.cfg](easy_handeye2/setup.cfg) | 將 easy_handeye2 的註冊執行檔安裝到 lib/easy_handeye2，供 ros2 run 尋找。 |
| [easy_handeye2/setup.py](easy_handeye2/setup.py) | 註冊校正 server 與兩個 rqt 指令，安裝 launch、GUI／plugin 資源、授權與雙語 README。 |
| [readme.md](readme.md) | 英文專案總覽、功能、相依分工、閱讀順序與完整檔案清單。 |
| [readme.zh-TW.md](readme.zh-TW.md) | 繁體中文專案總覽、功能、相依分工、閱讀順序與完整檔案清單。 |
| [ur3_bringup/CMakeLists.txt](ur3_bringup/CMakeLists.txt) | 宣告 ament_cmake 套件，安裝 launch、RViz 設定及雙語 README。 |
| [ur3_bringup/README.md](ur3_bringup/README.md) | 英文操作指南：實機／mock 啟動、launch 參數、RViz、SDK 排查與狀態檢查。 |
| [ur3_bringup/README.zh-TW.md](ur3_bringup/README.zh-TW.md) | 繁體中文操作指南：實機／mock 啟動、launch 參數、RViz、SDK 排查與狀態檢查。 |
| [ur3_bringup/config/moveit.rviz](ur3_bringup/config/moveit.rviz) | RViz 顯示設定：手臂、規劃場景、桌面、目標／路徑標記、影像、點雲與 TF。 |
| [ur3_bringup/launch/bringup.launch.py](ur3_bringup/launch/bringup.launch.py) | 整合 UR driver、MoveIt、RViz、桌面與相機；支援實機與 mock 模式。 |
| [ur3_bringup/launch/moveit.launch.py](ur3_bringup/launch/moveit.launch.py) | 引入官方 MoveIt launch，統一管理本專案的 RViz 與碰撞桌面，避免重複啟動。 |
| [ur3_bringup/package.xml](ur3_bringup/package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [ur3_moveit_scene/README.md](ur3_moveit_scene/README.md) | 英文操作指南：共用碰撞桌面與場景參數的使用方式。 |
| [ur3_moveit_scene/README.zh-TW.md](ur3_moveit_scene/README.zh-TW.md) | 繁體中文操作指南：共用碰撞桌面與場景參數的使用方式。 |
| [ur3_moveit_scene/package.xml](ur3_moveit_scene/package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [ur3_moveit_scene/resource/ur3_moveit_scene](ur3_moveit_scene/resource/ur3_moveit_scene) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [ur3_moveit_scene/setup.cfg](ur3_moveit_scene/setup.cfg) | 將 ur3_moveit_scene 的註冊執行檔安裝到 lib/ur3_moveit_scene，供 ros2 run 尋找。 |
| [ur3_moveit_scene/setup.py](ur3_moveit_scene/setup.py) | 註冊 virtual_table 指令，安裝 ROS 資源標記與雙語 README。 |
| [ur3_moveit_scene/ur3_moveit_scene/__init__.py](ur3_moveit_scene/ur3_moveit_scene/__init__.py) | 標示共用場景 Python 套件。 |
| [ur3_moveit_scene/ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/ur3_moveit_scene/virtual_table.py) | 將虛擬桌面加入 MoveIt Planning Scene，並發布場景更新與 RViz 標記。 |
| [ur3_pick_up/README.md](ur3_pick_up/README.md) | 英文操作指南：即時 AnyGrasp 夾取、啟動順序、TCP、參數、失敗處理與結束碼。 |
| [ur3_pick_up/README.zh-TW.md](ur3_pick_up/README.zh-TW.md) | 繁體中文操作指南：即時 AnyGrasp 夾取、啟動順序、TCP、參數、失敗處理與結束碼。 |
| [ur3_pick_up/config/grasp_tool.example.json](ur3_pick_up/config/grasp_tool.example.json) | Robotiq 工具設定範本：tool0、沿用程式 TCP 偏移、抓取軸向對應與最大開口 0.085 m。 |
| [ur3_pick_up/package.xml](ur3_pick_up/package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [ur3_pick_up/resource/ur3_pick_up](ur3_pick_up/resource/ur3_pick_up) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [ur3_pick_up/scripts/anygrasp_worker.py](ur3_pick_up/scripts/anygrasp_worker.py) | 容器內的單次點雲推論入口；載入已授權 SDK／權重，輸出候選與請求資訊，不控制手臂。 |
| [ur3_pick_up/setup.cfg](ur3_pick_up/setup.cfg) | 將 ur3_pick_up 的註冊執行檔安裝到 lib/ur3_pick_up，供 ros2 run 尋找。 |
| [ur3_pick_up/setup.py](ur3_pick_up/setup.py) | 註冊 pick_up 指令，安裝工具設定、容器推論 worker 與雙語 README。 |
| [ur3_pick_up/ur3_pick_up/__init__.py](ur3_pick_up/ur3_pick_up/__init__.py) | 標示即時夾取 Python 套件。 |
| [ur3_pick_up/ur3_pick_up/anygrasp_input.py](ur3_pick_up/ur3_pick_up/anygrasp_input.py) | 解析與驗證 AnyGrasp／GraspNet 候選及工具設定，處理深度、軸向與 TCP 姿態轉換。 |
| [ur3_pick_up/ur3_pick_up/anygrasp_task.py](ur3_pick_up/ur3_pick_up/anygrasp_task.py) | 串接擷取、推論與候選排序，依序嘗試可規劃姿態，確認後執行夾取、返回記錄的起始位置並等候放爪確認。 |
| [ur3_pick_up/ur3_pick_up/camera_capture.py](ur3_pick_up/ur3_pick_up/camera_capture.py) | 接收本次新的 PointCloud2，檢查時間與 frame，取得點雲對應的 TF 快照。 |
| [ur3_pick_up/ur3_pick_up/docker_client.py](ur3_pick_up/ur3_pick_up/docker_client.py) | 檢查與啟動既有容器，透過檔案與 docker cp／exec 送出推論請求並讀回結果。 |
| [ur3_pick_up/ur3_pick_up/live_input.py](ur3_pick_up/ur3_pick_up/live_input.py) | 點雲解碼、座標轉換、體素／點數篩選、圈選遮罩映射、候選排序及固定相機 TF 一致性檢查。 |
| [ur3_pick_up/ur3_pick_up/pick_up.py](ur3_pick_up/ur3_pick_up/pick_up.py) | pick_up 入口、ROS 參數、MoveIt 規劃／執行、TCP／路徑標記與 Robotiq socket 控制。 |
