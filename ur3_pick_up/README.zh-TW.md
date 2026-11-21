# UR3 即時 AnyGrasp 夾取

[English](README.md)

`ros2 run ur3_pick_up pick_up` 從相機取得新點雲，自動呼叫既有 Docker 的 AnyGrasp，依評分由高到低嘗試工作區內的有效 pose，選擇第一個去程與返回 home 都能規劃的姿態。一次啟動擷取一個場景，最多執行一個 pose；手臂移動前仍需確認。

## 工作區與 RViz

工作區以 **base_link 的 +Y 為前方**，水平大小 60×60 公分：

| 軸 | 範圍（公尺） |
| --- | --- |
| X | −0.30～+0.30 |
| Y | 0.00～+0.60 |
| Z | +0.02～+0.60 |

bringup 在實體與虛擬模式都會先顯示工作區預覽，不需啟動本程式、相機或 Docker。啟動本程式後，預覽節點會讓出發布，RViz 顯示這次任務參數的實際工作區；程式結束後恢復 bringup 預覽。

RViz 會顯示青色半透明盒、框線及範圍文字。標記直接併入目前 bringup 的 `/ur3_pick_up/target_and_path_markers`，與 TCP 目標、面向三軸及去回程路徑一起顯示。也可單獨加入 **MarkerArray**，Topic 設 `/ur3_pick_up/workspace`；Fixed Frame 用 `base_link`。程式運作時每秒重發，支援稍後才開啟的 RViz。工作區是選取範圍，不是障礙物或可達性保證；程序結束後既有 RViz 可能保留最後標記。

## 安裝與建置

主機使用 ROS 2 Jazzy、UR driver、MoveItPy、RealSense driver 與 `ur3_bringup`、`ur3_moveit_scene`、`auto_calibration` 與 `easy_handeye2`。PyTorch、CUDA、MinkowskiEngine 與 AnyGrasp 留在 Docker；主機 ROS Python 不需安裝模型套件。Docker 安裝與授權流程見 [Docker 講義](../docker+anygrasp/README.zh-TW.md)。

建置或執行前，先備妥下列相依：

| 相依項目 | 安裝／就緒條件 |
| --- | --- |
| ROS、官方 UR driver、MoveItPy 與 RealSense | 先讀[主機建置](../ROS2+driver/ur3_ros2_setup.zh-TW.md)，再依 [bringup](../ur3_bringup/README.zh-TW.md) 準備。 |
| Scene 與本套件 | 完整專案放入 `~/ur3_ws/src` 即可辨識，每個套件只保留一份。 |
| `easy_handeye2_msgs`、ArUco 與校正工具 | 由[主機講義第 4 節](../ROS2+driver/ur3_ros2_setup.zh-TW.md#handeye-message-dependency)下載訊息來源，並用 rosdep 安裝相依。 |
| 手臂與相機 TF | 完成校正並保持一份[手眼 publisher](../auto_calibration/README.zh-TW.md)運作；新安裝沒有該硬體配置的校正檔。 |
| Docker CLI、NVIDIA runtime、已授權 SDK 與 detection 權重 | 完成 [Docker 講義](../docker+anygrasp/README.zh-TW.md)第 3～7 節及 import／GPU、官方資料驗證。 |
| Robotiq TCP server | 在 UR 控制器啟用夾爪 socket 服務，設定正確 IP／port。本專案已包含 Python socket client，不需另開 ROS 1 夾爪節點。 |

官方資料推論成功只代表模型環境可用；即時執行還需要新相機資料與 TF。`plan_only` 略過硬體命令，但仍需要上述即時推論相依。

既有容器須為 `anygrasp-fixed`，已通過授權，內含：

- `/opt/anygrasp_sdk/grasp_detection/gsnet.so`：符合容器 Python 版本的 SDK。
- 同目錄的 `license/`：授權檔案。
- `log/checkpoint_detection.tar`：偵測權重。

不會重建容器、修改 MAC 或複製授權到原始碼。程式可啟動已存在但停止的容器。

完成[主機講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節後，本套件已建置，不需再執行 apt、rosdep 或 colcon。接著備妥 Docker、授權／權重與校正 TF，再執行下列啟動流程。

## 啟動

終端 1：在手臂控制端啟用 External Control，再啟動手臂、相機點雲、MoveIt、RViz 與虛擬桌面：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

若相機 driver 已在其他終端運作，加 `launch_camera:=false`。不要重複啟動相機。

終端 2：維持唯一的手眼校正 publisher；若尚未啟動：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

終端 3：啟動即時夾取。`sudo -v` 在這個終端取得 Docker 使用權；ROS 節點本身使用一般使用者執行，不要整支用 sudo 執行。

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

若目前使用者已可直接執行 `docker`，可省略 `sudo -v`，並加 `--ros-args -p docker_use_sudo:=false`。

只看工作區、推論及規劃，完全不下達手臂／夾爪命令：

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args -p plan_only:=true
```

此時相機、校正 TF、MoveIt 與桌面仍須運作；從目前手臂狀態擷取及規劃，不先移動到 home。

## 執行前檢查與操作提示

保持 bringup 與手眼 publisher 兩個終端運作。在準備執行 pick_up 的終端，可先確認控制器與點雲：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 control list_controllers
ros2 topic info /camera/camera/depth/color/points
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

確認 `scaled_joint_trajectory_controller` 為 `active`、點雲有 publisher，且 TF 能持續輸出。`tf2_echo` 會持續執行，檢查後按 Ctrl+C 結束，再執行 pick_up。RViz 的點雲與手臂應先對齊，工作區框線應位於實際要抓取的桌面範圍。

| 操作時機 | 畫面／提示 | 操作與結果 |
| --- | --- | --- |
| 初始 home 規劃成功 | `Move the arm to initial state 'home'? Enter y:` | 檢查 RViz 初始路徑；輸入 `y` 才移動。其他回答取消整次任務。 |
| home 到位 | 擷取新點雲、顯示場景／工作區點數與紀錄路徑 | 等待一次 Docker 推論；目前不需輸入候選索引。 |
| 推論及篩選成功 | 候選索引、score、width、TCP 相機座標 | 程式依分數由高到低嘗試有效 pose，直到某個 pose 的去回程都規劃成功。 |
| 去回程均成功 | `Confirm arm/gripper execution? Enter y:` | 檢查目標面向與兩段路徑；輸入 `y` 才開始候選任務。其他回答取消後續動作；初始 home 可能已完成。 |
| 完成 | 已返回 home 與開爪釋放結果 | 本次流程結束。若要換物體再抓一次，重新執行啟動指令，會重新擷取點雲及推論。 |
| 任一階段失敗 | 階段名稱與失敗原因 | 停止後續步驟；依訊息與 exchange 紀錄排查後重新啟動。 |

`plan_only:=true` 不會出現這兩個執行確認提示，也不會移到初始 home；它仍會嘗試取得點雲、呼叫 Docker 並從目前手臂狀態規劃。

## 任務流程

1. 顯示工作區，檢查共用桌面、Docker 容器與 SDK／權重路徑。
2. 規劃初始 home，顯示路徑並詢問 `Move the arm to initial state 'home'? Enter y:`。
3. 到位後等待 0.5 秒，再擷取新 PointCloud2；只接受擷取開始後、年齡不超過 `cloud_max_age` 的樣本。
4. 依訊息真實 `header.frame_id` 及擷取時間 TF 轉成 `camera_color_optical_frame`。以同一時間的相機→base TF 決定工作區；不直接改 frame 名稱。
5. 過濾無效點／相機深度、體素降採樣並限制點數。保留範圍外場景點供 SDK 檢查夾爪碰撞，將工作區布林遮罩一同送入容器。工作區點數不足就停止，避免 SDK 將全 false 遮罩視為不限定區域。
6. 容器只推論一次，使用 `dense_grasp=false`、`collision_detection=true`、`region_steering=工作區遮罩`，再 NMS 及評分排序。無需 SAM，但不會辨識物體名稱。
7. 主機再次檢查候選的開口、旋轉與位置。原始抓取中心及加上 depth 後的 TCP 均須在工作區內；將有效 pose 依 score 由高到低排序，逐一嘗試去程與返回 home 的規劃。某個 pose 的任一段規劃失敗，就換次高分 pose；直到兩段都成功才選定。全部有效 pose 都失敗時，顯示英文 `All candidate poses failed to plan.`，結束候選任務。規劃嘗試期間不移動手臂，也不操作夾爪。
8. 所有 pose 共用同一次點雲與擷取時的 TF；每次嘗試前及選定後都檢查 TF。選定的 pose 兩段成功才詢問 `Confirm arm/gripper execution? Enter y:`。回答 y 後再檢查 TF，更新超過 3 mm 或 0.5° 就取消。
9. 執行到候選姿態 → 開爪 POS=0 → 閉爪 POS=255 → 返回 home → 開爪 POS=0 釋放物體。各階段失敗會停止後續動作並標示階段。夾爪動作失敗時不會自動回 home。只有返回 home 執行成功後才開爪；若回程失敗，就不下達釋放命令。若 home 開爪時因接觸而提前停止，會回報釋放階段失敗。

推論結果屬於擷取時的靜止場景，確認期間請保持相機與物體不動。本流程使用固定相機，不適用相機隨手臂移動的配置。SDK 的點雲碰撞篩選只處理抓取幾何；點雲不會自動變成 MoveIt 全手臂路徑的障礙物，路徑仍依目前 MoveIt 場景檢查。

## 座標與 TCP

SDK 接收相機座標 N×3、公尺點雲。Optical +X 向影像右、+Y 向下、+Z 沿鏡頭向前；工作區另外在 base_link 定義。終端 TCP 數值用 optical frame 顯示。

工具設定預設安裝套件內的 `config/grasp_tool.example.json`，依目前 Robotiq 2F-85 設定：TCP = tool0 +Z 0.174 m，兩者軸向相同；開口上限 0.085 m。運算為：

```text
p_TCP_camera = translation + depth * R_grasp[:, 0]
R_TCP_camera = R_grasp * grasp_to_tcp_rotation
T_base_tool0 = T_base_camera * T_camera_grasp_tip * T_grasp_tip_tool0
```

TCP 偏移沿旋轉後的工具 +Z 補償一次。`width` 用來篩選，動作仍為全開／全閉，不換算成對應 POS。POS=255 是全閉命令；夾到物體或完成行程時實際 POS 可以不同。

## ROS 參數

使用 `--ros-args -p 名稱:=值`。工作區修改範例：

```bash
ros2 run ur3_pick_up pick_up --ros-args \
  -p workspace_min:='[-0.30, 0.0, 0.02]' \
  -p workspace_max:='[0.30, 0.60, 0.60]'
```

| 參數 | 預設 | 用途 |
| --- | --- | --- |
| `workspace_min` / `workspace_max` | 上表 XYZ | base_frame 內的選取盒，兩組三元素陣列，公尺。 |
| `pointcloud_topic` | `/camera/camera/depth/color/points` | 新 PointCloud2 來源；依 header frame 正式轉換。 |
| `cloud_timeout` / `cloud_max_age` | `10.0` / `1.0` | 等待秒數／樣本可接受年齡。 |
| `voxel_size` | `0.005` | 體素間距，公尺。 |
| `max_points` / `min_roi_points` | `120000` / `50` | 完整場景點數上限／降採樣後工作區最少點數。 |
| `max_camera_depth` | `2.0` | Optical Z 最遠範圍，公尺。 |
| `docker_container` / `docker_use_sudo` | `anygrasp-fixed` / `true` | 既有容器名稱／以 sudo -n 呼叫 Docker。 |
| `docker_sdk_dir` | `/opt/anygrasp_sdk/grasp_detection` | 容器 SDK／license 工作目錄。 |
| `checkpoint_path` | SDK 的 `log/checkpoint_detection.tar` 絕對路徑 | 容器內權重。 |
| `inference_timeout` | `120.0` | 推論最長秒數；逾時嘗試結束該次容器 worker。 |
| `gripper_height` | `0.03` | SDK 碰撞篩選的手指高度，公尺。 |
| `exchange_dir` | `~/anygrasp_docker/exchange` | 每次唯一工作目錄，保留點雲、TF、結果及推論紀錄。 |
| `plan_only` | `false` | true 不移動手臂、不操作夾爪；仍取點雲／推論／規劃。 |
| `grasp_tool_config` | 安裝套件的工具 JSON | 抓取軸與 TCP 軸對應、工具轉換、實體開口上限。 |
| `base_frame` / `tool_link` | `base_link` / `tool0` | 工作區及內部規劃 frame／規劃末端。目前工具限定 tool0。 |
| `tcp_offset` | `0.174` | tool0 +Z 至 TCP 距離，公尺；須符合實際硬體。 |
| `planning_group` | `ur_manipulator` | MoveIt 關節群組。 |
| `robot_ip` / `gripper_port` | `192.168.86.7` / `63352` | 夾爪 TCP server；bringup 手臂 IP 另外設定。 |
| `gripper_backend` | `tcp` | `none` 只略過夾爪，確認後手臂仍會移動。 |
| `max_velocity_scaling_factor` / `max_acceleration_scaling_factor` | `0.1` / `0.1` | MoveIt 速度／加速度倍率，(0,1]。 |

OMPL 使用 RRTConnect、規劃時間 5 秒、規劃嘗試設定 10 次。工作區不可代表所有姿態均能到達。

若 6 GB GPU 推論遇到記憶體不足，可先以較少點數、較粗體素做不移動的檢查：

```bash
sudo -v
ros2 run ur3_pick_up pick_up --ros-args \
  -p plan_only:=true \
  -p max_points:=60000 \
  -p voxel_size:=0.008 \
  -p inference_timeout:=180.0
```

降採樣會影響小物體的幾何細節與候選結果；工作區點數仍需滿足 `min_roi_points`。


## 排錯與驗證

- 無點雲：檢查 bringup、相機連線與 topic；使用 `ros2 topic list` 確認實際名稱。
- 工作區無點：先看 RViz 框線、點雲與手臂是否對齊，檢查手眼校正與 TF，勿只改 frame 字串。
- Docker permission denied／sudo requires password：在啟動節點的同一終端先 `sudo -v`；若等待過久憑證過期，重新執行。
- CUDA OOM：降低 `max_points`、增加 `voxel_size` 或縮小 `max_camera_depth`；空區域仍會拒絕推論。
- 去程／回程規劃失敗：依分數改試下一個有效 pose，直到成功或全部用完。全部失敗顯示 `All candidate poses failed to plan.`；初始 home 可能已經完成。若確認後已開始執行，動作失敗會停止後續流程，不會再改試其他 pose。
- 每次輸出保留在 exchange 的 UUID 子目錄：`cloud.npz`（points、region、capture metadata）、`capture.json`（TF／工作區）、`grasps.json`（成功結果）、`inference.log`。容器暫存會清除；主機紀錄需自行定期清理。
- `Deleting MoveItCpp` 後 Segmentation fault 是既有原生清理問題；不能據此判定任務已成功，須看各階段結果。

執行前可用 `plan_only:=true` 檢查新點雲擷取、Docker 推論、工作區與去回程規劃。它仍需要相機、校正 TF、MoveIt 與桌面，但不會下達手臂／夾爪命令。真實相機、GPU、校正精度及手臂執行需依現場狀況確認。

SDK 介面依據：[AnyGrasp 官方使用說明](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)。

## 程式結束碼

`pick_up` 只有任務完成，或 `plan_only` 模式的去回程都規劃成功時回傳 `0`。點雲擷取、推論、候選規劃或執行失敗回傳 `1`；拒絕執行確認也回傳 `1`。啟動／設定錯誤回傳 `2`。Ctrl+C 或互動輸入關閉回傳 `130`。

原生函式庫崩潰由作業系統另外回報；上述應用程式結束碼不會掩蓋 MoveItCpp segmentation fault。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [README.md](README.md) | 英文操作指南：即時 AnyGrasp 夾取、工作區、啟動順序、TCP、參數、失敗處理與結束碼。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：即時 AnyGrasp 夾取、工作區、啟動順序、TCP、參數、失敗處理與結束碼。 |
| [config/grasp_tool.example.json](config/grasp_tool.example.json) | Robotiq 工具設定範本：tool0、沿用程式 TCP 偏移、抓取軸向對應與最大開口 0.085 m。 |
| [package.xml](package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [resource/ur3_pick_up](resource/ur3_pick_up) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [scripts/anygrasp_worker.py](scripts/anygrasp_worker.py) | 容器內的單次點雲推論入口；載入已授權 SDK／權重，輸出候選與請求資訊，不控制手臂。 |
| [setup.cfg](setup.cfg) | 將 ur3_pick_up 的註冊執行檔安裝到 lib/ur3_pick_up，供 ros2 run 尋找。 |
| [setup.py](setup.py) | 註冊 pick_up 指令，安裝工具設定、容器推論 worker 與雙語 README。 |
| [ur3_pick_up/__init__.py](ur3_pick_up/__init__.py) | 標示即時夾取 Python 套件。 |
| [ur3_pick_up/anygrasp_input.py](ur3_pick_up/anygrasp_input.py) | 解析與驗證 AnyGrasp／GraspNet 候選及工具設定，處理深度、軸向與 TCP 姿態轉換。 |
| [ur3_pick_up/anygrasp_task.py](ur3_pick_up/anygrasp_task.py) | 串接擷取、推論與候選排序，依序嘗試可規劃姿態，確認後執行夾取與返回 home／開爪。 |
| [ur3_pick_up/camera_capture.py](ur3_pick_up/camera_capture.py) | 接收本次新的 PointCloud2，檢查時間與 frame，取得點雲對應的 TF 快照。 |
| [ur3_pick_up/docker_client.py](ur3_pick_up/docker_client.py) | 檢查與啟動既有容器，透過檔案與 docker cp／exec 送出推論請求並讀回結果。 |
| [ur3_pick_up/live_input.py](ur3_pick_up/live_input.py) | 點雲解碼、座標轉換、工作區／體素／點數篩選、候選排序及固定相機 TF 一致性檢查。 |
| [ur3_pick_up/pick_up.py](ur3_pick_up/pick_up.py) | pick_up 入口、ROS 參數、MoveIt 規劃／執行、TCP／路徑標記與 Robotiq socket 控制。 |
