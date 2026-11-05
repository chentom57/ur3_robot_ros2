# UR3 基座座標夾取程式

此 ROS 2 套件接受 `base_link` 座標中的目標點，為 `tool0` 規劃去程，並預先規劃回到 MoveIt named state `up` 的路徑。兩段路徑都規劃成功後，程式會在 RViz 顯示目標與路徑，並等待使用者確認後才移動或操作夾爪。

## 功能

- 輸入以公尺為單位的 `base_link` XYZ 目標，並沿用目前 `tool0` 姿態。
- 每次提示前在終端顯示目前 `tool0` 座標，並在 RViz 標出位置。
- 先規劃目前位置至目標的路徑，再從去程終點預先規劃返回 `up` 的路徑。任一規劃失敗都不會開始移動。
- 規劃成功後，在 RViz 顯示目標及兩段路徑；輸入 `y` 確認後才執行。
- 抵達目標後停頓一秒、關閉夾爪、返回 `up`，再打開夾爪。
- 可取樣指定高度的 XY 逆向運動學（IK）切片，並列出有或沒有 IK 解的位置區間。

## 系統需求

- Ubuntu 與 ROS 2 Jazzy。
- UR 機器人 bringup、`ur_moveit_config`、MoveIt 2，以及可發布機器人狀態的 TF。
- SRDF 中包含 `ur_manipulator` 規劃群組與名為 `up` 的 named state。
- 使用實體夾爪時，Robotiq TCP 服務須已啟動，且設定的 IP 與 port 可連線。

## 放入 ROS 2 工作區並建置

將此資料夾放在工作區的 `src` 目錄下，並保留資料夾名稱 `ur3_base_pick`：

```bash
cp -r /path/to/robotic\ control/ur3_base_pick ~/ur3_ws/src/
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-select ur3_base_pick
source install/setup.bash
```

每次開啟新終端執行程式前，載入 ROS 與工作區環境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_base_pick base_coordinate_pick
```

請先在其他終端啟動機器人驅動程式、MoveIt 與必要的夾爪 TCP 服務。程式會連接目前執行中的 ROS 2 系統，不會自行啟動機器人 bringup。

## 操作

程式會持續顯示目前 `tool0` 在 `base_link` 中的座標，並接受以下輸入：

| 輸入 | 動作 |
| --- | --- |
| `x y z` | 規劃前往指定位置及返回 `up` 的路徑。數值可用空格或逗號分隔，例如 `-0.22 0.25 0.56` 或 `-0.22, 0.25, 0.56`。 |
| `r` | 再次列印目前 `tool0` 座標；不會移動。 |
| `m` | 輸入切片高度 `z`，列印 XY IK 區間並發布工作範圍標記。 |
| `q` | 結束程式。 |

去程和回程都規劃成功後，請先在 RViz 檢查路徑及周圍空間。輸入 `y` 才會開爪、移至目標、夾取並返回 `up`。其他輸入會取消，手臂與夾爪都不會動作。

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
  -p gripper_backend:=tcp
```

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `base_frame` | `base_link` | 輸入座標所屬的基座座標系 |
| `tool_link` | `tool0` | MoveIt 規劃的末端連結 |
| `planning_group` | `ur_manipulator` | MoveIt 規劃群組 |
| `robot_ip` | `192.168.86.7` | Robotiq TCP 服務 IP |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` 控制實體夾爪；`none` 略過夾爪命令 |
| `workspace_map_min` | `-0.60` | 切片 x、y 掃描下界（公尺） |
| `workspace_map_max` | `0.60` | 切片 x、y 掃描上界（公尺） |
| `workspace_map_step` | `0.05` | XY 取樣間距（公尺） |

設定 `gripper_backend:=none` 時，程式仍會規劃並執行手臂路徑，但會略過開爪、夾取與放開動作。
