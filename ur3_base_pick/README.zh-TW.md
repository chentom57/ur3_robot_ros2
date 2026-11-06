# UR3 基座座標夾取程式

此 ROS 2 套件接受 `base_link` 座標中的目標點，為 `tool0` 規劃去程，並預先規劃回到 MoveIt named state `up` 的路徑。兩段路徑都規劃成功後，程式會在 RViz 顯示目標與路徑，並等待使用者確認後才移動或操作夾爪。

虛擬桌面由共用的 `ur_moveit_config` launch 設定，並非由本夾取節點建立。安裝與設定方式請見 [`ur3_moveit_scene/README.zh-TW.md`](../ur3_moveit_scene/README.zh-TW.md)。此夾取程式啟動時會等待 `/planning_scene` 收到 `ur3_virtual_table_collision`；若缺少此物件就會拒絕規劃。桌面發布器會在新的規劃用戶端連線時重新發布場景更新，讓獨立的 MoveItPy 規劃器收到相同桌面幾何。

## 功能

- 輸入以公尺為單位的 `base_link` XYZ 目標，並沿用目前 `tool0` 姿態。
- 每次提示前在終端顯示目前 `tool0` 座標，並在 RViz 標出位置。
- 先規劃目前位置至目標的路徑，再從去程終點預先規劃返回 `up` 的路徑。任一規劃失敗都不會開始移動。
- 規劃成功後，在 RViz 顯示目標及兩段路徑；輸入 `y` 確認後才執行。
- 規劃軌跡的預設關節速度與加速度上限為機器人設定值的 10%；可透過 ROS 參數調整。
- 使用 TCP 夾爪時，流程開始先閉爪；抵達目標後開爪、等待 0.5 秒、閉爪，再等待 0.5 秒後返回 `up`，並保持閉爪。
- 可取樣指定高度的 XY 逆向運動學（IK）切片，並列出有或沒有 IK 解的位置區間。

## 系統需求

- Ubuntu 與 ROS 2 Jazzy。
- UR 機器人 bringup、`ur_moveit_config`、MoveIt 2，以及可發布機器人狀態的 TF。
- SRDF 中包含 `ur_manipulator` 規劃群組與名為 `up` 的 named state。
- 使用實體夾爪時，Robotiq TCP 服務須已啟動，且設定的 IP 與 port 可連線。

## 建置與載入環境

將此資料夾放在工作區的 `src` 目錄下，並保留資料夾名稱 `ur3_base_pick`：

```bash
cp -r /path/to/robotic\ control/ur3_base_pick ~/ur3_ws/src/
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-select ur3_base_pick --symlink-install
source install/setup.bash
```

## 啟動

請先在另一個終端啟動 UR driver 與 MoveIt。本程式連接目前執行中的 ROS 2 系統，不會自行啟動或切換機器人硬體。請使用 `ur3_bringup` 套件選擇硬體模式並載入共用虛擬桌面。程式啟動時會等待 `/planning_scene` 中出現 `ur3_virtual_table_collision`；若桌面物件不存在，程式會拒絕規劃。bringup 參數請見 [`ur3_bringup/README.zh-TW.md`](../ur3_bringup/README.zh-TW.md)，共用桌面設定請見 [`ur3_moveit_scene/README.zh-TW.md`](../ur3_moveit_scene/README.zh-TW.md)。

在另一個終端載入 ROS 與工作區環境後執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_base_pick base_coordinate_pick
```

`gripper_backend` 預設為 `tcp`。若要避免向實體夾爪傳送指令，請設為 `none`：

```bash
ros2 run ur3_base_pick base_coordinate_pick --ros-args -p gripper_backend:=none
```

使用 Mock Hardware 時請設為 `none`。若要實際夾取，請確認 Robotiq TCP 服務可透過設定的 `robot_ip` 與 `gripper_port` 連線；參數詳見下方表格。

## 操作

程式會持續顯示目前 `tool0` 在 `base_link` 中的座標，並接受以下輸入：

| 輸入 | 動作 |
| --- | --- |
| `x y z` | 規劃前往指定位置及返回 `up` 的路徑。數值可用空格或逗號分隔，例如 `-0.22 0.25 0.56` 或 `-0.22, 0.25, 0.56`。 |
| `r` | 再次列印目前 `tool0` 座標；不會移動。 |
| `m` | 輸入切片高度 `z`，列印 XY IK 區間並發布工作範圍標記。 |
| `q` | 結束程式。 |

去程和回程都規劃成功後，請先在 RViz 檢查路徑及周圍空間。輸入 `y` 後，程式會先閉爪再移至目標；抵達後開爪、等待 0.5 秒、閉爪，再等待 0.5 秒後返回 `up`，並在 `up` 保持閉爪。設定 `gripper_backend:=none` 時會略過所有夾爪命令。輸入其他內容會取消，手臂與夾爪都不會動作。

## RViz 視覺化

將 RViz 的 **Fixed Frame** 設為 `base_link`，並新增 **MarkerArray** 顯示項目：

- `/ur3_base_coordinate_pick/target_and_path_markers`：紅色球體為目標點、藍線為去程、黃線為返回 `up` 的路徑。
- `/ur3_base_coordinate_pick/workspace_map`：顯示目前 `tool0` 位置；輸入 `m` 後顯示所選高度切片。綠色格表示該取樣點找到 IK 解，紅色格表示本次取樣未找到 IK 解。

工作範圍切片會固定目前 `tool0` 姿態，並且只檢查 IK。它不保證能找到無碰撞路徑，也不代表完整工作範圍邊界。最終仍須依據 MoveIt 對指定去程與回程的規劃結果判斷。

## 參數

覆寫參數範例：

```bash
ros2 run ur3_base_pick base_coordinate_pick --ros-args \
  -p robot_ip:=192.168.86.7 \
  -p gripper_port:=63352 \
  -p gripper_backend:=tcp \
  -p max_velocity_scaling_factor:=0.1 \
  -p max_acceleration_scaling_factor:=0.1
```

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `base_frame` | `base_link` | 輸入座標所屬的基座座標系 |
| `tool_link` | `tool0` | MoveIt 規劃的末端連結 |
| `planning_group` | `ur_manipulator` | MoveIt 規劃群組 |
| `robot_ip` | `192.168.86.7` | Robotiq TCP 服務 IP |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` 控制實體夾爪；`none` 略過夾爪命令 |
| `max_velocity_scaling_factor` | `0.1` | 關節最大速度縮放值；數值越小手臂越慢。有效範圍：`(0, 1]`。 |
| `max_acceleration_scaling_factor` | `0.1` | 關節最大加速度縮放值；數值越小加速越平緩。有效範圍：`(0, 1]`。 |
| `workspace_map_min` | `-0.60` | 切片 x、y 掃描下界（公尺） |
| `workspace_map_max` | `0.60` | 切片 x、y 掃描上界（公尺） |
| `workspace_map_step` | `0.05` | XY 取樣間距（公尺） |

設定 `gripper_backend:=none` 時，程式仍會規劃並執行手臂路徑，但會略過所有夾爪命令。

速度與加速度縮放參數會限制規劃軌跡相對於機器人模型設定值的上限。UR 控制器的安全設定與速度滑桿也可能進一步限制實際速度。
