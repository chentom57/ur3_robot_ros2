# UR3 即時相機 AnyGrasp 夾取

[English](README.md)

執行 `ros2 run ur3_pick_up pick_up` 後，程式會記錄手臂當下關節位置作為任務起點與回程目標，接著開啟即時彩色影像視窗。使用者點選矩形的兩個對角，圈選一個目標物。GUI 會在推論、規劃及移動期間保持開啟；程式會先詢問是否執行，手臂回到記錄的起始位置後，再等候 GUI 放爪指令。

矩形遮罩會映射到點雲並引導 AnyGrasp 產生候選姿態。完整篩選後的場景點雲也會提供給 SDK 做碰撞篩選。主機依夾爪最大開口篩選並依分數排序，再由 MoveIt 規劃去程與回程並檢查手臂／桌面碰撞。程式會依序嘗試候選，直到兩段路徑都規劃成功。

## 相依與建置

使用 ROS 2 Jazzy、UR driver、MoveItPy、RealSense ROS、`ur3_bringup`、`ur3_moveit_scene`、`auto_calibration` 與 `easy_handeye2`。完成[主機建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節；完整專案建置已包含本套件。PyTorch、CUDA、MinkowskiEngine 與 AnyGrasp 留在 Docker 容器中，授權 SDK 與權重依 [Docker 講義](../docker+anygrasp/README.zh-TW.md)準備。

既有的 `anygrasp-fixed` 容器需包含 `gsnet.so`、授權檔及 `log/checkpoint_detection.tar`。UR 控制器需啟用 Robotiq socket service。保持一個手眼 TF publisher 運作；新安裝環境需使用自己的校正檔。

## 啟動系統

終端機 1：在手臂啟用 External Control，啟動 driver、相機、MoveIt、RViz 與共用虛擬桌面：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

若相機已在其他終端啟動，加入 `launch_camera:=false`。

終端機 2：若尚未有校正 publisher，啟動一個：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

終端機 3：必要時先驗證 Docker 權限，再啟動夾取程式：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

若帳號可直接使用 Docker，可省略 `sudo -v` 並設定 `docker_use_sudo:=false`。

只擷取、推論及規劃，不送出手臂／夾爪命令：

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args -p plan_only:=true
```

`plan_only` 仍需要即時相機、校正 TF、MoveIt、Docker 與碰撞桌面。手臂保持原位，程式會記錄目前關節狀態作為規劃起點與返回目標。

## 操作流程

1. 程式記錄目前關節位置，不會先移動手臂。
2. GUI 顯示即時彩色影像。點選矩形兩個對角圈住一個物體；Enter 接受、右鍵或 `U` 撤銷一點、`R` 重設、Esc 取消。
3. 程式擷取新的同步影像與點雲，利用相機內參將影像遮罩映射至點雲。完整篩選後的場景點雲提供碰撞篩選；圈選遮罩引導候選生成。這是人工圈選，不是語意分割；請將不相關的鄰近物體排除在矩形外。
4. 開口超過夾爪設定的候選會被移除，其餘依分數排序。MoveIt 依序規劃去程及返回記錄起始關節位置的路徑。失敗姿態會略過；若全部失敗，程式回報統計且不執行軌跡。
5. GUI 詢問是否執行。手臂移動前先打開夾爪，接著移至選定姿態、閉合夾爪並返回記錄的起始位置。
6. 夾爪保持閉合，直到使用者在 GUI 確認放爪。取消則維持閉合。流程完成或停止後關閉 GUI。

每次任務只擷取一次點雲與相機 TF。推論和確認期間請保持相機及目標物靜止。AnyGrasp 使用點雲進行夾取碰撞篩選；點雲不會被加入 MoveIt 的全手臂碰撞場景。MoveIt 依現有規劃場景檢查手臂路徑，包含共用虛擬桌面。

## 座標與 TCP

SDK 接收公尺制、N×3 的相機 optical 座標點雲：+X 為影像右方、+Y 向下、+Z 朝鏡頭前方。`base_frame` 用於 MoveIt 規劃；終端機顯示的 TCP 位置使用 optical frame。

安裝的 `config/grasp_tool.example.json` 描述目前 Robotiq 2F-85 設定：tool0 +Z TCP 偏移 0.174 m，最大開口 0.085 m。請依實際工具確認設定。

```text
p_TCP_camera = translation + depth * R_grasp[:, 0]
R_TCP_camera = R_grasp * grasp_to_tcp_rotation
T_base_tool0 = T_base_camera * T_camera_grasp_tip * T_grasp_tip_tool0
```

## 主要 ROS 參數

使用 `--ros-args -p 名稱:=值` 傳入覆寫值。

| 參數 | 預設值 | 用途 |
|---|---|---|
| `pointcloud_topic` | `/camera/camera/depth/color/points` | 點雲來源 |
| `color_image_topic` | `/camera/camera/color/image_raw` | 圈選使用的原始彩色影像 |
| `color_camera_info_topic` | `/camera/camera/color/camera_info` | 將圈選映射至點雲所需的相機內參 |
| `cloud_timeout` / `cloud_max_age` | `10.0` / `1.0` | 等待逾時／資料最大年齡，秒 |
| `voxel_size` | `0.005` | 降採樣體素大小，公尺 |
| `max_points` / `min_roi_points` | `120000` / `50` | 全場景點數上限／降採樣後圈選區域最低點數 |
| `max_camera_depth` | `2.0` | optical Z 最大距離，公尺 |
| `docker_container` / `docker_use_sudo` | `anygrasp-fixed` / `true` | 既有容器／Docker 權限方式 |
| `docker_sdk_dir` | `/opt/anygrasp_sdk/grasp_detection` | 容器內 SDK 與授權目錄 |
| `checkpoint_path` | SDK `log/checkpoint_detection.tar` | 容器內偵測權重 |
| `inference_timeout` | `120.0` | 推論時間上限，秒 |
| `gripper_height` | `0.03` | SDK 碰撞篩選使用的手指高度，公尺 |
| `exchange_dir` | `~/anygrasp_docker/exchange` | 每次執行的點雲、metadata、結果及 log 目錄 |
| `plan_only` | `false` | 擷取、推論及規劃，但不送出機器人命令 |
| `grasp_tool_config` | 安裝的工具 JSON | 工具轉換、夾取軸向與開口限制 |
| `base_frame` / `tool_link` | `base_link` / `tool0` | 規劃座標系與末端；目前設定需使用 tool0 |
| `tcp_offset` | `0.174` | tool0 +Z 至 TCP，公尺；需符合實際工具 |
| `planning_group` | `ur_manipulator` | MoveIt 規劃群組 |
| `robot_ip` / `gripper_port` | `192.168.86.7` / `63352` | 夾爪控制器位址與 port |
| `gripper_backend` | `tcp` | 設為 `none` 可略過夾爪命令 |
| `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` | `0.1` / `0.1` | MoveIt 軌跡速度／加速度比例，範圍 (0,1] |

OMPL 使用 RRTConnect，規劃時間 5 秒，設定 10 次規劃嘗試。去程與回程皆須規劃成功，但仍不保證實際執行一定成功。

若 GPU 記憶體不足，可在 plan-only 下減少點數上限並增加體素：

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args \
  -p plan_only:=true -p max_points:=60000 \
  -p voxel_size:=0.008 -p inference_timeout:=180.0
```

降採樣可能移除小物體細節；圈選區在篩選及降採樣後仍需至少有 `min_roi_points` 個點。

## 問題排查

- 沒有點雲：檢查相機連線與 `ros2 topic list` 中的 topic。
- 圈選點數過少：放大目標矩形、確認影像畫面，並確認物體在設定的深度範圍內。
- PointCloud2 row-step 錯誤：檢查目前 publisher 與 topic；解碼錯誤會列出尺寸、point step、row step 及資料長度。
- Docker 權限錯誤：在同一終端先執行 `sudo -v`。
- CUDA 記憶體不足：降低 `max_points`、增加 `voxel_size` 或降低 `max_camera_depth`。
- 規劃失敗：程式會依分數嘗試下一個候選。全部失敗時，檢查候選與規劃 log、手臂狀態、TF、桌面高度及 MoveIt 場景。
- 每次請求會儲存在 `exchange_dir` 的 UUID 資料夾：`cloud.npz`、`capture.json`、成功推論時的 `grasps.json` 及 `inference.log`。
- `Deleting MoveItCpp` 後發生 segmentation fault 屬於原生程式清理問題；依程式輸出的任務結果判斷動作是否完成。

## 結束碼

任務完成或 plan-only 規劃成功回傳 `0`；任務／規劃失敗或拒絕執行回傳 `1`；啟動／設定錯誤回傳 `2`；Ctrl+C 或互動輸入關閉回傳 `130`。

<a id="file-inventory"></a>
## 檔案名稱與用途

| 檔案名稱 | 功用 |
|---|---|
| [README.md](README.md) | 英文操作指南。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南。 |
| [config/grasp_tool.example.json](config/grasp_tool.example.json) | 工具與 TCP 設定範例。 |
| [package.xml](package.xml) | ROS 套件資訊與執行相依。 |
| [resource/ur3_pick_up](resource/ur3_pick_up) | ament 索引套件標記。 |
| [scripts/anygrasp_worker.py](scripts/anygrasp_worker.py) | 容器推論 worker，不控制機器人。 |
| [setup.cfg](setup.cfg) | 安裝 `ros2 run` 使用的執行檔。 |
| [setup.py](setup.py) | 註冊執行檔並安裝套件資源。 |
| [ur3_pick_up/anygrasp_input.py](ur3_pick_up/anygrasp_input.py) | 解析候選並轉換至設定的工具／TCP 座標。 |
| [ur3_pick_up/anygrasp_task.py](ur3_pick_up/anygrasp_task.py) | 協調圈選、推論、規劃與確認後的夾取流程。 |
| [ur3_pick_up/camera_capture.py](ur3_pick_up/camera_capture.py) | 擷取新點雲及對應相機 TF。 |
| [ur3_pick_up/docker_client.py](ur3_pick_up/docker_client.py) | 與既有 Docker 容器交換推論請求及結果。 |
| [ur3_pick_up/live_input.py](ur3_pick_up/live_input.py) | 解碼點雲、映射圈選遮罩與排序候選。 |
| [ur3_pick_up/pick_up.py](ur3_pick_up/pick_up.py) | ROS 入口、MoveIt、RViz 標記與夾爪控制。 |
| [ur3_pick_up/target_selection.py](ur3_pick_up/target_selection.py) | 即時影像圈選與操作確認 GUI。 |
