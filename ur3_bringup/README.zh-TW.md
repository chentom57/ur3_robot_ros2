# UR3 Bringup

[English](README.md)

`ur3_bringup` 啟動 UR3 driver、MoveIt、RViz、共用虛擬碰撞桌面，以及可選的 RealSense 相機。

官方 UR driver 與 `ur_moveit_config` 從 GitHub 取得。本套件的 `launch/moveit.launch.py` 引入官方 MoveIt，再加入桌面與隨附的 `config/moveit.rviz`，不需修改官方 driver。可透過 bringup 啟動，或先啟動 driver 後分開執行：

```bash
ros2 launch ur3_bringup moveit.launch.py ur_type:=ur3
```

此整合會停用官方 launch 的 RViz，以及已客製化官方 launch 可能加入的桌面，再各啟動一份本專案的 RViz 與桌面發布器。RViz 設定包含手臂、場景、目標／路徑標記、彩色影像、點雲與相機／base 三軸。

## 安裝與建置

先完成[主機與原始碼建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節。該流程會建置本套件與場景套件；完成後直接依下節啟動，不需再次安裝或建置。

| 相依項目 | 準備方式 |
| --- | --- |
| ROS 2 Jazzy、rosdep、colcon | 主機講義第 1 節。 |
| `ur_robot_driver`、`ur_moveit_config` | 主機講義第 4 節的官方 GitHub `jazzy` 來源。 |
| `realsense2_camera` 與系統 SDK | 官方來源 tag 4.59.1，搭配主機講義第 2 節的系統 SDK。 |
| `ur3_moveit_scene` | 已隨本專案提供。 |
| MoveIt、RViz 與 launch 相依 | 主機講義第 4 節的 rosdep 指令統一安裝。 |

整份專案的建置也需要 `easy_handeye2_msgs`，來源取得方式已列在主機講義。mock bringup 的執行不需要校正結果、Docker 或 AnyGrasp 授權。每個新終端都先 source ROS 與工作區。

## 實體手臂

設定手臂控制器的外部控制程式與網路連線，再執行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

保持此終端運作。設定的機型為 UR3，實機模式預設啟動相機及其點雲、RViz 與桌面。若相機 driver 已獨立運作：

```bash
ros2 launch ur3_bringup bringup.launch.py launch_camera:=false
```

## 模擬硬體

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py \
  use_mock_hardware:=true launch_camera:=false
```

Driver 使用 mock hardware，不連接實體手臂。`launch_camera:=auto` 時，模擬硬體模式會自動停用相機啟動。

## RViz 顯示

RViz 的 Fixed Frame 設為 `base_link`。隨附設定會顯示手臂、規劃場景、共用碰撞桌面、目標與規劃路徑、相機影像、點雲及 TF。目標／路徑 MarkerArray topic 為 `/ur3_pick_up/target_and_path_markers`。

## Launch 參數

使用 `名稱:=值`，不加 `--ros-args -p`。

| 參數 | 預設值 | 用途 |
| --- | --- | --- |
| `robot_ip` | `192.168.86.7` | 實體 UR 控制器 IP。 |
| `use_mock_hardware` | `false` | `true` 使用模擬硬體，`false` 連接實體手臂。 |
| `virtual_table_enabled` | `true` | 將共用碰撞桌面發布至 MoveIt 場景。 |
| `virtual_table_z` | `-0.01` | 桌面頂面在 `base_link` 的高度，公尺，需符合實際安裝。 |
| `launch_camera` | `auto` | `auto`：實機開相機、模擬不開；`true`／`false` 強制指定。 |
| `launch_rviz` | `true` | 啟動 MoveIt RViz。 |

由本套件啟動的相機使用 `pointcloud.enable=true`，本套件沒有獨立的點雲 launch 選項。桌面為 4×4 m、厚 0.02 m 的碰撞板，頂面在 `virtual_table_z`，物件 ID 為 `ur3_virtual_table_collision`。

覆寫參數範例：

```bash
ros2 launch ur3_bringup bringup.launch.py \
  robot_ip:=192.168.86.7 launch_camera:=true \
  virtual_table_z:=-0.01 launch_rviz:=true
```

## 相機 SDK 來源與重建

RealSense SDK 統一使用 RealSense apt 來源提供的系統套件 `librealsense2`、`librealsense2-dev`。目前環境使用 SDK 2.59.1；原始碼版 RealSense ROS 4.59.x 需要 SDK 2.59.0 以上。SDK 來源與安裝方式見[環境建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)。

執行主機講義的 `rosdep` 指令前，先安裝上述系統 SDK。指令加上 `--skip-keys librealsense2`，因為 ROS 的這個相依鍵會另外安裝 `ros-jazzy-librealsense2`，在 `/opt/ros/jazzy` 產生第二套 SDK。跳過該鍵代表 SDK 改由系統套件管理，不會自動替你安裝 SDK。

檢查安裝來源：

```bash
dpkg-query -W librealsense2 librealsense2-dev
apt-cache policy ros-jazzy-librealsense2
```

若同時已安裝 ROS 版 SDK，先用 `apt-get --simulate remove ros-jazzy-librealsense2` 檢查移除清單，再執行 `sudo apt remove ros-jazzy-librealsense2`。請依該電腦的實際清單判斷，尤其是使用二進位版 RealSense ROS 套件的環境。本專案的原始碼版相機 driver 使用系統 SDK。

SDK 版本更新或安裝來源調整後，重新建置相機 driver：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
cd ~/ur3_ws
colcon build --packages-select realsense2_camera --symlink-install \
  --cmake-force-configure --cmake-clean-first \
  --cmake-args -DBUILD_TESTING=OFF
source ~/ur3_ws/install/setup.bash
```

此流程直接使用相機 driver 的一般 CMake 相依搜尋，不需本專案額外的 SDK 輔助檔，也不需修改官方相機原始碼。重建後重新啟動 bringup，確認編譯／執行 SDK 版本一致、深度／點雲有新資料，且 `camera_link → camera_color_optical_frame` 可查詢，再啟動手眼 publisher。版本一致仍需另外確認相機串流正常。

## 檢查啟動狀態

```bash
ros2 control list_controllers
ros2 topic list
```

檢查控制器狀態與相機 topics。RViz 的 Fixed Frame 使用 `base_link`，可查看手臂模型與碰撞桌面。結束時在 bringup 終端按 Ctrl+C。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [CMakeLists.txt](CMakeLists.txt) | 宣告 ament_cmake 套件，安裝 launch、RViz 設定及雙語 README。 |
| [README.md](README.md) | 英文操作指南：實機／mock 啟動、launch 參數、RViz、SDK 排查與狀態檢查。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：實機／mock 啟動、launch 參數、RViz、SDK 排查與狀態檢查。 |
| [config/moveit.rviz](config/moveit.rviz) | RViz 顯示設定：手臂、規劃場景、桌面、目標／路徑標記、影像、點雲與 TF。 |
| [launch/bringup.launch.py](launch/bringup.launch.py) | 整合 UR driver、MoveIt、RViz、桌面與相機；支援實機與 mock 模式。 |
| [launch/moveit.launch.py](launch/moveit.launch.py) | 引入官方 MoveIt launch，統一管理本專案的 RViz 與碰撞桌面，避免重複啟動。 |
| [package.xml](package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
