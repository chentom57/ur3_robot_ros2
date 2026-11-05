# UR3 座標夾取程式

本文件說明 `ur3_pick_demo` Python 原型的用途、流程、安裝與操作。此程式與 ROS 2 環境建置分開；請先依照 [ROS 2 建置指南](../ur3_ros2_setup.md) 設定 ROS 2、UR 驅動程式、MoveIt、相機與 Easy HandEye 2。

## 1. 程式功能

程式接受相機座標系中的三維點 `(x, y, z)`，將其轉換到機器人基座座標系，再使用 MoveIt 規劃手臂接近、夾取及抬起的動作。若使用實體 Robotiq 夾爪，程式會透過 TCP socket 傳送開合指令。

這是用來驗證手眼 TF 與基本動作流程的原型。輸入是三維座標，不是影像像素 `(u, v)`。程式目前不會辨識物體、將像素轉成三維座標、估算夾取方向，也不會加入物體碰撞模型。

## 2. 運作方式

### 座標轉換

Easy HandEye 2 的校正結果必須由其 TF 發布器載入並發布到 ROS TF 樹。程式不會直接讀取 `.calib` 檔，而是查詢從 `camera_color_optical_frame` 到 `base_link` 的轉換，並將輸入點轉換到 `base_link`。

輸入點會被當作 `tool0` 的目標位置，而非物體中心。輸入座標應是工具中心預計抵達的位置。程式會從 TF 讀取目前 `tool0` 姿態，並在移動時沿用該姿態。

### 手臂動作

程式使用 MoveItPy，預設規劃群組為 `ur_manipulator`，工具連結為 `tool0`。動作順序如下：

1. 將輸入點轉換到 `base_link`，並計算高於目標 10 公分的接近點。
2. 若使用 TCP 夾爪後端，先打開 Robotiq 夾爪。
3. 規劃並執行接近點軌跡。
4. 規劃並執行前往目標點的軌跡。
5. 關閉夾爪，再規劃並執行返回接近點的抬起軌跡。

程式會先詢問是否開始，並在每段手臂軌跡執行前要求輸入 `y`。確認前請檢查 RViz 中的路徑、工作範圍，以及手臂周圍空間。

### 夾爪連線

預設的 `tcp` 後端會連線至 `robot_ip` 與 `gripper_port`（預設為 `192.168.86.7:63352`），啟動夾爪並下達開啟位置 `0`、關閉位置 `255`。這些是控制器的位置命令，不代表程式能確認物體已成功夾起。

設定 `gripper_backend:=none` 可略過實體夾爪命令，但手臂軌跡仍會送至 MoveIt 控制器。使用模擬硬體時，請以 `use_mock_hardware:=true` 啟動 UR 驅動程式，並確認 MoveIt 也連接至模擬控制器。此設定不會模擬夾爪。

## 3. 前置條件

- UR 驅動程式、MoveIt 與 RealSense 相機皆已啟動。
- Easy HandEye 2 TF 發布器使用正確的校正名稱執行中，且 TF 樹中的相機座標系與 `base_link` 相連。
- 相機座標以公尺為單位，並且相對於程式設定的相機座標系有效。
- 若操作實體設備，夾爪 TCP 服務須可連線，工作空間內也不得有人員或障礙物進入手臂活動範圍。

Easy HandEye 2 發布器範例（請將名稱換成已儲存的校正名稱）：

```bash
ros2 launch easy_handeye2 publish.launch.py name:=ur3_realsense_eob
```

## 4. 安裝套件

將 `ur3_pick_demo/` 資料夾複製到 ROS 工作區的 `src` 目錄。以下以 `~/ur3_ws` 為例：

```bash
cp -r /path/to/ur3_pick_demo ~/ur3_ws/src/
cd ~/ur3_ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src --rosdistro jazzy -y
colcon build --symlink-install
```

每個執行程式的終端都要載入 ROS 與工作區環境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
```

首次建置後，若套件原始碼有修改，請重新執行 `colcon build --symlink-install`。

## 5. 啟動與操作

建議使用不同終端分別啟動驅動程式／MoveIt／相機、Easy HandEye TF 發布器與夾取程式。每個終端都要載入 ROS 與工作區環境。

在新終端執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run ur3_pick_demo coordinate_pick
```

依提示輸入相機座標系中的 `x y z`（公尺），例如：

```text
Camera x y z> 0.32 -0.08 0.45
```

也可以用逗號分隔。輸入 `q`、`quit` 或 `exit` 可結束程式。程式會列出轉換後的 `base_link` 座標並要求確認；每段手臂軌跡執行前也會再次要求輸入 `y`。若座標、姿態或路徑有疑慮，請勿確認執行。

## 6. 參數

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | Robotiq TCP 服務的 IP 位址 |
| `gripper_port` | `63352` | Robotiq TCP port |
| `gripper_backend` | `tcp` | `tcp` 控制實體夾爪；`none` 略過夾爪命令 |
| `base_frame` | `base_link` | 機器人基座座標系 |
| `camera_frame` | `camera_color_optical_frame` | 輸入點所屬的相機座標系 |
| `tool_link` | `tool0` | MoveIt 目標工具連結 |
| `planning_group` | `ur_manipulator` | MoveIt 規劃群組 |
| `approach_height` | `0.10` | 目標上方的接近與抬起高度（公尺） |

覆寫參數範例：

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args \
  -p camera_frame:=camera_color_optical_frame \
  -p approach_height:=0.10 \
  -p gripper_backend:=tcp
```

使用模擬手臂並略過實體夾爪命令：

```bash
ros2 run ur3_pick_demo coordinate_pick --ros-args -p gripper_backend:=none
```

## 7. 常見問題

### TF 查詢失敗

確認 Easy HandEye 2 發布器仍在執行、校正名稱正確，且實際 TF 座標系名稱符合 `base_frame` 和 `camera_frame`。可執行 `ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame` 檢查轉換。

### MoveIt 無法規劃

確認 UR 驅動程式與 MoveIt 已啟動，且 `ur_manipulator` 與 `tool0` 符合目前 UR 設定；並確認目標可達。若規劃失敗，程式不會執行該段軌跡。

### 夾爪連線或命令失敗

檢查 `robot_ip`、port、網路連線及 Robotiq 控制器狀態。若只要測試手臂規劃，可設定 `gripper_backend:=none`；此設定仍會控制手臂。

## 8. 後續整合物體辨識

輸入迴圈與 `pick_camera_point(point_camera)` 分開。未來的感知節點可將影像偵測結果轉為相機座標系中的三維夾取點，再呼叫相同流程。之後也可加入夾取方向估算、碰撞物件及抓取結果檢查。目前請勿直接把像素座標輸入程式。

## RViz 目標標記

輸入的相機座標點會轉換至 `base_link`，並以紅色 `GRASP` 球體發布；綠色球體代表接近點。兩個標記都會顯示輸入的相機座標標籤。在 RViz 中將 **Fixed Frame** 設為 `base_link`，新增 **Marker** 顯示項目，Topic 設為 `/ur3_coordinate_pick/target_markers`。標記會保留到輸入下一個點為止。
