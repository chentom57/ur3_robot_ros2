# UR3 座標夾取程式：原理與操作說明

本文件獨立說明 `ur3_pick_demo` Python 原型的用途、運作流程、安裝方式與操作方法。它不屬於 ROS2 環境建置講義；請先依照 `ur3_ros2_setup.md` 建立 ROS 2、UR 驅動、MoveIt、相機與 Easy HandEye 2 環境。

## 1. 程式能做什麼

程式讓使用者輸入一個相機座標系中的三維位置 `(x, y, z)`，將位置轉換到機器人基座座標系，再由 MoveIt 規劃手臂接近、夾取及抬起的動作。使用實體 Robotiq 夾爪時，程式透過 TCP socket 發送開合指令。

這是方便驗證手眼 TF 和基本動作流程的原型。輸入是「三維座標」，不是影像像素 `(u, v)`。目前也沒有物體辨識、像素轉三維、夾取姿態推算或物體碰撞模型。

## 2. 程式原理

### 座標轉換

Easy HandEye 2 儲存的校正結果需要由其 TF 發布器載入並發布到 ROS TF tree。程式不會直接讀取 `.calib` 檔，而是透過 TF 查詢從 `camera_color_optical_frame` 到 `base_link` 的轉換，將輸入點換算到 `base_link`。

輸入點會被當成 `tool0` 的目標位置。它不是物體中心；實際使用時，輸入座標應已經是希望工具中心抵達的位置。程式從 TF 取得目前 `tool0` 的姿態，移動時沿用該姿態。

### 手臂動作

程式使用 MoveItPy，預設規劃組為 `ur_manipulator`，工具連結為 `tool0`。動作順序如下：

1. 將輸入點轉成 `base_link` 座標，並計算高出目標 10 公分的接近點。
2. 開啟 Robotiq 夾爪（若使用 TCP 夾爪後端）。
3. 規劃並執行接近點軌跡。
4. 規劃並執行目標點軌跡。
5. 關閉夾爪，再規劃並執行回到接近點的抬起軌跡。

程式會先詢問是否開始，並在每一段手臂軌跡執行前再次要求輸入 `y`。請先檢查 RViz 顯示的軌跡、工作範圍及手臂周圍空間，再確認執行。

### 夾爪連線

預設使用 `tcp` 後端，連到 `robot_ip` 的 `gripper_port`（預設 `192.168.86.7:63352`），啟動夾爪並下達開啟位置 `0`、關閉位置 `255`。這些是控制器命令的位置值，不代表可直接用來判斷夾爪一定夾到物體；目前流程沒有可靠的抓取成功判定。

設定 `gripper_backend:=none` 可跳過實體夾爪指令，但手臂軌跡仍會送給 MoveIt 的控制器執行。若是使用 mock hardware，需先以 `use_mock_hardware:=true` 啟動 UR driver，並確認 MoveIt 也連到該 mock 控制器。這不會自動模擬夾爪。

## 3. 前置條件

操作前請確認：

- UR driver、MoveIt 與 RealSense 相機已啟動，相關終端保持執行。
- Easy HandEye 2 的 TF 發布器已使用正確的校正名稱啟動，且 TF tree 中存在程式指定的 camera frame 與 `base_link`。
- 相機座標使用公尺，且輸入點相對於程式指定的相機 frame 有效。
- 實體操作時，夾爪 TCP 服務可連線，且工作空間內沒有障礙物或人員進入運動範圍。

Easy HandEye 2 發布器範例（將名稱換成實際儲存的校正名稱）：

```bash
ros2 launch easy_handeye2 publish.launch.py name:=ur3_realsense_eob
```

## 4. 安裝程式

將隨附的 `ur3_pick_demo/` 資料夾複製到 ROS workspace 的 `src` 目錄。以下假設 workspace 是 `~/ur3_ws`：

```bash
cp -r /path/to/ur3_pick_demo ~/ur3_ws/src/
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

每個要執行程式的終端都要載入 ROS 與 workspace 環境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

若套件已經建置完成，後續不需重複安裝；修改套件程式碼後重新執行 `colcon build --symlink-install`。

## 5. 啟動與操作

建議各自使用終端啟動 driver/MoveIt/相機、Easy HandEye TF 發布器及座標夾取程式。每個終端先載入 ROS 與 workspace 環境。

在新的終端執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_pick_demo coordinate_pick
```

依提示輸入相機座標的 `x y z`（公尺），例如：

```text
相機座標 x y z> 0.32 -0.08 0.45
```

也可以用逗號分隔。輸入 `q`、`quit` 或 `exit` 結束程式。程式會顯示換算後的 `base_link` 座標並要求確認；接著每段手臂軌跡執行前都會再次提示輸入 `y`。若座標、姿態或路徑看起來不正確，請勿確認執行。

## 6. 參數

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | Robotiq TCP 服務所在 IP |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` 控制實體夾爪；`none` 略過夾爪控制 |
| `base_frame` | `base_link` | 機器人基座座標 frame |
| `camera_frame` | `camera_color_optical_frame` | 輸入點所屬的相機 frame |
| `tool_link` | `tool0` | MoveIt 目標工具連結 |
| `planning_group` | `ur_manipulator` | MoveIt 規劃組 |
| `approach_height` | `0.10` | 目標上方的接近與抬起高度，單位公尺 |

覆寫參數範例：

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args \
  -p camera_frame:=camera_color_optical_frame \
  -p approach_height:=0.10 \
  -p gripper_backend:=tcp
```

使用 mock arm 並略過實體夾爪時：

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args -p gripper_backend:=none
```

## 7. 常見問題

### 找不到 TF

確認 Easy HandEye 2 發布器仍在執行、校正名稱正確，並檢查實際 TF frame 名稱是否與 `base_frame`、`camera_frame` 參數相符。可用 `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 檢查轉換是否可查詢。

### MoveIt 無法規劃

確認 UR driver 與 MoveIt 已啟動、`ur_manipulator` 規劃組及 `tool0` 連結與目前 UR 設定一致，並檢查目標是否在可達範圍內。規劃失敗時程式不會執行該段軌跡。

### 夾爪連線或命令失敗

檢查 `robot_ip`、port、網路連線及 Robotiq 控制器狀態。若只測試手臂規劃，可設 `gripper_backend:=none`；這仍會操作手臂控制器。

## 8. 未來物體辨識整合

目前輸入迴圈與 `pick_camera_point(point_camera)` 夾取方法分開。未來辨識節點可先將影像中的目標轉為相機 frame 的三維夾取點，再呼叫相同流程；之後再擴充夾取姿態估計、障礙物場景與抓取結果檢查。現階段請不要把像素座標直接當成此程式的輸入。
