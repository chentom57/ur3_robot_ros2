# MoveIt 共用虛擬桌面

此套件會將大型水平碰撞板加入共用的 MoveIt Planning Scene。它由 `ur_moveit_config/launch/ur_moveit.launch.py` 啟動，因此 `move_group`、RViz、夾取程式及其他規劃用戶端都能使用同一個桌面。碰撞板為 4 公尺 × 4 公尺，可涵蓋 UR3 工作範圍；桌面不由夾取節點建立或管理。

預設碰撞板的上表面位於 `base_link` 座標系的 `z = -0.01 m`，也就是該座標系原點下方 1 公分。請量測實體桌面相對於 `base_link` 的高度，並視需要調整。碰撞幾何為厚 2 公分的方板，向下延伸。

## 安裝至 `ur3_ws`

將此套件複製到工作區的原始碼目錄：

```bash
cp -r /path/to/robotic\ control/ur3_moveit_scene ~/ur3_ws/src/
```

在 `~/ur3_ws/src/Universal_Robots_ROS2_Driver` 的 Universal Robots 驅動原始碼中，進行以下兩項修改：

1. 在 `ur_moveit_config/package.xml` 加入 `<exec_depend>ur3_moveit_scene</exec_depend>`。
2. 在 `ur_moveit_config/launch/ur_moveit.launch.py` 加入 `ur3_moveit_scene` 的 `virtual_table` 執行程式，並與 MoveIt 一起啟動。目前工作區中的檔案已包含這些修改。

建置套件與 MoveIt 設定：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-up-to ur_moveit_config --symlink-install
source ~/ur3_ws/install/setup.bash
```

## 啟動

使用一般 MoveIt 啟動指令即可。虛擬桌面預設啟用：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

可在啟動時設定桌面上表面相對於 `base_link` 的高度，例如：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

若要停用，設定 `virtual_table_enabled:=false`。

整合式 `ur3_bringup` launch 也會啟動此 MoveIt 設定，並傳遞桌面高度與啟用參數：

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

分開開啟多個終端的 helper 也會呼叫同一個 `ur_moveit_config` launch，因此預設仍會載入桌面。

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
