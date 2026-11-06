# UR3 Bringup

此套件會啟動 UR3 driver、MoveIt、MoveIt RViz 與共用虛擬桌面，也可啟動 RealSense 相機。可在 launch 指令中使用 `參數名稱:=值` 設定啟動方式。

## 建置與載入環境

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ur3_ws
colcon build --packages-up-to ur3_bringup --symlink-install
source ~/ur3_ws/install/setup.bash
```

## 啟動

使用預設參數啟動 bringup：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py
```

### 可用參數

| 參數 | 預設值 | 說明 |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | 連接實體硬體時使用的 UR 控制器 IP。 |
| `use_mock_hardware` | `false` | 設為 `true` 使用 UR ROS 2 driver 的 Mock Hardware；`false` 則連接 `robot_ip` 指定的控制器。 |
| `virtual_table_enabled` | `true` | 將共用虛擬桌面碰撞物件加入 MoveIt Planning Scene。 |
| `virtual_table_z` | `-0.01` | 虛擬桌面上表面相對於 `base_link` 的高度，單位為公尺。 |
| `launch_camera` | `auto` | `auto` 會在 Mock Hardware 模式關閉相機、在實體硬體模式啟動相機；可用 `true` 或 `false` 強制指定。 |
| `launch_rviz` | `true` | 啟動 MoveIt RViz。 |

可在 launch 指令中覆寫一個或多個參數，例如：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py \
  use_mock_hardware:=true \
  launch_camera:=false \
  virtual_table_z:=-0.04
```

虛擬桌面是尺寸為 4 m × 4 m、厚度 0.02 m 的碰撞板，其上表面高度由 `virtual_table_z` 設定。預設值是在 `base_link` 中的 `-0.01 m`；請將參數設為該座標系中量測到的實際桌面高度。MoveIt 規劃時會檢查此碰撞物件。若要在某次啟動時停用桌面，可設 `virtual_table_enabled:=false`。

可用以下指令確認軌跡控制器已啟動：

```bash
ros2 control list_controllers
```

完整環境建置方式請參考[ROS 2 UR3 設定講義（英文版）](../ur3_ros2_setup.md)或[繁體中文版](../ur3_ros2_setup.zh-TW.md)。

