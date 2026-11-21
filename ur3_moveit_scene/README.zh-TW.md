# MoveIt 共用虛擬桌面

此套件會將大型水平碰撞板加入共用的 MoveIt Planning Scene。它由 `ur3_bringup/launch/moveit.launch.py` 啟動，因此 `move_group`、RViz、夾取程式及其他規劃用戶端都能使用同一個桌面。碰撞板為 4 公尺 × 4 公尺，可涵蓋 UR3 工作範圍；桌面不由夾取節點建立或管理。

預設碰撞板的上表面位於 `base_link` 座標系的 `z = -0.01 m`，也就是該座標系原點下方 1 公分。請量測實體桌面相對於 `base_link` 的高度，並視需要調整。碰撞幾何為厚 2 公分的方板，向下延伸。

## 安裝與建置

完成[主機與原始碼建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節即可。完整專案包含本套件，`rosdep` 會安裝 rclpy、NumPy 與 geometry／visualization／MoveIt／shape 訊息相依，`colcon` 會一併建置；此處不需重複安裝。

執行整合式啟動需要 [bringup](../ur3_bringup/README.zh-TW.md) 的官方 UR／MoveIt 相依。獨立的場景與工作區發布器不需要相機、校正結果、Docker 或 AnyGrasp。每個套件只保留一份，不需修改官方 UR driver。

## 啟動

使用本專案的 MoveIt 啟動指令即可。虛擬桌面預設啟用：

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

可在啟動時設定桌面上表面相對於 `base_link` 的高度，例如：

```bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

若要停用，設定 `virtual_table_enabled:=false`。

整合式 `ur3_bringup` launch 也會啟動此 MoveIt 設定，並傳遞桌面高度與啟用參數：

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

碰撞板為 4 公尺 × 4 公尺，可涵蓋 UR3 工作範圍；上表面高度由 `virtual_table_z` 設定。套件會在 `/moveit_virtual_table/marker` 發布相同尺寸的半透明藍色方板標記。隨附的 MoveIt RViz 設定已加入此標記顯示項目。

## Topic 與參數

| 名稱 | 預設值 | 說明 |
|---|---:|---|
| Launch 參數 `virtual_table_enabled` | `true` | 是否啟動共用桌面發布器 |
| Launch 參數 `virtual_table_z` | `-0.01` | 桌面上表面在 `base_link` 中的高度，公尺 |
| Node 參數 `base_frame` | `base_link` | 定義桌面的座標系 |
| Node 參數 `table_extent` | `4.0` | 碰撞板邊長，公尺 |
| Node 參數 `table_thickness` | `0.02` | 上表面以下的板厚，公尺 |
| Planning Scene topic | `/planning_scene` | MoveIt 場景監控器接收共用碰撞物件更新的 topic |
| RViz 標記 topic | `/moveit_virtual_table/marker` | 顯示碰撞板 |

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [README.md](README.md) | 英文操作指南：共用碰撞桌面、工作區顯示與場景參數的使用方式。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：共用碰撞桌面、工作區顯示與場景參數的使用方式。 |
| [package.xml](package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [resource/ur3_moveit_scene](resource/ur3_moveit_scene) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [setup.cfg](setup.cfg) | 將 ur3_moveit_scene 的註冊執行檔安裝到 lib/ur3_moveit_scene，供 ros2 run 尋找。 |
| [setup.py](setup.py) | 註冊 virtual_table 與 workspace_preview 指令，安裝 ROS 資源標記與雙語 README。 |
| [ur3_moveit_scene/__init__.py](ur3_moveit_scene/__init__.py) | 標示共用場景 Python 套件。 |
| [ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/virtual_table.py) | 將虛擬桌面加入 MoveIt Planning Scene，並發布場景更新與 RViz 標記。 |
| [ur3_moveit_scene/workspace.py](ur3_moveit_scene/workspace.py) | 共用工作區範圍、點位包含判斷與工作區 MarkerArray 幾何。 |
| [ur3_moveit_scene/workspace_preview.py](ur3_moveit_scene/workspace_preview.py) | bringup 的獨立工作區預覽節點；執行夾取時讓出標記發布，結束後恢復。 |
