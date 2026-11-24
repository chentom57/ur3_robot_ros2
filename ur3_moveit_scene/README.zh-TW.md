# MoveIt 共用虛擬碰撞桌面

此套件會將水平碰撞板加入共用的 MoveIt Planning Scene。`ur3_bringup/launch/moveit.launch.py` 會啟動發布器，讓 `move_group`、RViz 與規劃用戶端使用同一個桌面。夾取節點不負責建立或管理桌面。

預設桌面上表面位於 `base_link` 的 `z = -0.01 m`。請量測實體桌面相對於 `base_link` 的高度，並視需要調整 `virtual_table_z`。碰撞板為 4 公尺 × 4 公尺、厚 2 公分，從上表面向下延伸。套件也會發布相同尺寸的半透明藍色 RViz 標記。

## 建置

完成[主機與原始碼建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節即可。本套件已包含在完整專案建置流程中，不需另外安裝。

## 啟動

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3 launch_rviz:=true
```

可在 `base_link` 中設定桌面高度，例如：

```bash
ros2 launch ur3_bringup moveit.launch.py \
  ur_type:=ur3 launch_rviz:=true virtual_table_z:=-0.035
```

整合式 bringup 也會傳遞相同設定：

```bash
ros2 launch ur3_bringup bringup.launch.py virtual_table_z:=-0.035
```

適當情況下可設定 `virtual_table_enabled:=false`，不加入碰撞板。

## Topic 與參數

| 名稱 | 預設值 | 說明 |
|---|---:|---|
| Launch 參數 `virtual_table_enabled` | `true` | 是否啟動共用桌面發布器 |
| Launch 參數 `virtual_table_z` | `-0.01` | 桌面上表面在 `base_link` 中的高度，公尺 |
| Node 參數 `base_frame` | `base_link` | 定義桌面的座標系 |
| Node 參數 `table_extent` | `4.0` | 桌面邊長，公尺 |
| Node 參數 `table_thickness` | `0.02` | 上表面以下的厚度，公尺 |
| Planning Scene topic | `/planning_scene` | 共用碰撞物件更新 |
| RViz 標記 topic | `/moveit_virtual_table/marker` | 顯示碰撞板 |

<a id="file-inventory"></a>
## 檔案名稱與用途

| 檔案名稱 | 功用 |
|---|---|
| [README.md](README.md) | 英文共用碰撞桌面指南。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文共用碰撞桌面指南。 |
| [package.xml](package.xml) | ROS 套件資訊與執行相依。 |
| [resource/ur3_moveit_scene](resource/ur3_moveit_scene) | ament 索引套件標記。 |
| [setup.cfg](setup.cfg) | 安裝 `lib/ur3_moveit_scene` 執行檔。 |
| [setup.py](setup.py) | 註冊 `virtual_table` 並安裝套件資源與指南。 |
| [ur3_moveit_scene/__init__.py](ur3_moveit_scene/__init__.py) | Python 套件標記。 |
| [ur3_moveit_scene/virtual_table.py](ur3_moveit_scene/virtual_table.py) | 將共用桌面加入 MoveIt 並發布 RViz 標記。 |
