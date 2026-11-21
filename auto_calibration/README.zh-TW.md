# UR3 自動手眼校正與 TF 發布

[English](README.md)

`auto_calibration` 自動移動 UR3，透過 easy_handeye2 服務採樣及計算校正，並管理 `ur3_pick_up` 使用的 TF。預設為固定式相機（`eye_on_base`）；採樣時 ArUco 標記必須牢固固定於工具。

## 安裝與建置

完成[主機與原始碼建置講義](../ROS2+driver/ur3_ros2_setup.zh-TW.md)第 1～4 節即可；完整工作區會一併建置 `auto_calibration` 與 `easy_handeye2`，不需重複安裝。

| 相依項目 | 準備方式 |
| --- | --- |
| ROS 2 Jazzy、UR driver、MoveItPy、RealSense 與系統 SDK | 依主機講義準備。 |
| `easy_handeye2` 與 `easy_handeye2_msgs` 0.5.0 | 通用校正服務隨專案提供；訊息來源依主機講義下載。 |
| ArUco、TF 與 Python 函式庫 | 由 package.xml 宣告，交由主機講義的 rosdep 指令安裝。 |

`auto_calibration` 負責 UR3 自動移動、tracker 啟動、流程確認與相機根座標 TF 發布；[easy_handeye2](../easy_handeye2/README.zh-TW.md)負責採樣、校正計算、儲存與手動 GUI。自動流程只啟動背景校正服務，不啟動手動 GUI。兩者不需 Docker、AnyGrasp 授權或權重。

確認必要入口：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 pkg executables auto_calibration
ros2 interface show easy_handeye2_msgs/srv/TakeSample
ros2 pkg executables aruco_opencv
```

Tracker 清單需包含 `aruco_tracker_autostart`。各套件名稱只保留一份。

## 啟動校正

在手臂控制端啟用外部控制程式，啟動 driver、MoveIt、相機與 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch ur3_bringup bringup.launch.py robot_ip:=192.168.86.7
```

保持相機固定，標記牢固固定於工具，並在預定移動範圍內維持可見。Marker size 必須是標記實際邊長，單位公尺。起始手臂姿態需保留旋轉與平移空間。

另開終端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 run auto_calibration calibrate
```

每次啟動會先排查並停止本機、目前使用者、同一 `ROS_DOMAIN_ID` 內可辨識的舊自動校正流程、Easy HandEye server／GUI、校正 publisher，以及使用相同影像 topic 的 ArUco tracker，包含前次異常結束留下的子程序。先以 SIGINT 正常關閉，逾時才送 SIGTERM；不會停止 bringup、相機 driver 或手臂控制器。

清理後會確認相關程序、服務、ROS 節點及資源鎖已釋放，再建立新的 TF buffer 並啟動 tracker。如果程序再次出現、無法停止、或仍有遠端／無法辨識的衝突，本次直接失敗，不開始採樣。空的鎖定檔不代表仍有程序，不會透過刪除鎖定檔強行繞過。校正檔與備份不會被清除。

啟動清理會停止原本獨立運作的校正 publisher；儲存後仍會詢問是否發布，回答 `y` 才重新發布。既有 RViz 可能保留先前收到的靜態 TF，停止 publisher 不等於清除所有 listener 的快取。

程式會啟動 tracker，等待 `camera_color_optical_frame → marker_582`，再詢問 `Marker detected. Start automatic calibration? [y/n]:`。回答 `y` 代表開始自動移動與採樣；預檢後不會逐點再次詢問。

流程先清除 server 採樣清單並確認為零，再預規劃所有目標及返回起始姿態的路徑，通過後逐段重新規劃、執行。共 15 個樣本：工具各軸 ±20°、±10° 旋轉，以及手臂 base 各軸 +0.04 m 平移。每次移動後等待 1.5 秒再採樣，每個目標最多規劃 10 次。回到起始姿態後計算並儲存校正。這些動作設定是程式常數，沒有提供為 CLI 選項。

儲存後詢問 `Calibration saved. Start TF publishing? [y/n]:`。回答 `y` 會重讀該校正既有的 publisher，或啟動新的 publisher 並保持終端運作；回答 `n` 僅儲存。Ctrl+C 結束本流程管理的程序；獨立啟動的 publisher 不受此流程管理。

## 校正指令參數

使用 `--選項 值`，不是 ROS 的 `-p`：

```bash
ros2 run auto_calibration calibrate \
  --name ur3_realsense_eob \
  --calibration-type eye_on_base \
  --robot-base-frame base_link \
  --robot-effector-frame tool0 \
  --tracking-base-frame camera_color_optical_frame \
  --tracking-root-frame camera_link \
  --tracking-marker-frame marker_582 \
  --image-topic /camera/camera/color/image_raw \
  --marker-size 0.04 \
  --marker-dict ARUCO_ORIGINAL \
  --startup-timeout 180
```

| 選項 | 預設值 | 用途 |
| --- | --- | --- |
| `--name` | `ur3_realsense_eob` | 校正檔與發布服務名稱。 |
| `--calibration-type` | `eye_on_base` | `eye_on_base`：相機固定、標記在工具上；`eye_in_hand`：相機在工具上、標記固定，需符合實體安裝。 |
| `--robot-base-frame` | `base_link` | 手臂參考座標。 |
| `--robot-effector-frame` | `tool0` | 手臂末端座標。 |
| `--tracking-base-frame` | `camera_color_optical_frame` | Tracker 表達標記姿態所用座標。 |
| `--tracking-root-frame` | `camera_link` | 發布時連接至手臂樹的相機根座標。 |
| `--tracking-marker-frame` | `marker_582` | 偵測標記的 TF 名稱，須符合 ID 與字典。 |
| `--image-topic` | `/camera/camera/color/image_raw` | Tracker 使用的相機影像 topic。 |
| `--marker-size` | `0.04` | 標記實際邊長，公尺，必須為正且有限。 |
| `--marker-dict` | `ARUCO_ORIGINAL` | ArUco 字典。 |
| `--startup-timeout` | `180.0` | 偵測標記後等待校正服務與必要 TF 的秒數；初始標記偵測會持續等待至找到或中斷。 |

可使用 `--help` 查詢選項。內部 worker 選項不需手動使用。

## 發布已儲存校正

發布者需要運作中的相機先提供內部 `camera_link → camera_color_optical_frame` TF。若出現 `No camera-internal TF`，先檢查相機串流與啟動日誌；該錯誤本身不代表校正檔遺失。相機 SDK 編譯／執行版本不一致的處理方式見 [bringup 說明](../ur3_bringup/README.zh-TW.md)。

同一校正的 publisher 若已運作，不要重複啟動；否則：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
ros2 launch auto_calibration publish.launch.py \
  name:=ur3_realsense_eob tracking_root_frame:=camera_link
```

| Launch 參數 | 預設值 | 用途 |
| --- | --- | --- |
| `name` | 必填 | 要讀取的校正名稱。 |
| `tracking_root_frame` | `camera_link` | Publisher 連接的相機根座標。 |

執行 pick up 時保持 publisher 與相機 driver 運作。校正檔儲存於 `~/.ros2/easy_handeye2/calibrations/<name>.calib`；覆寫儲存會產生時間戳備份。檔案記錄校正 tracking frame 的姿態，publisher 利用相機內部 TF 換算並連接根座標。本 eye-on-base 配置發布 `base_link → camera_link`，RealSense 負責 `camera_link → ... → camera_color_optical_frame`。Publisher 只負責校正邊，並限制本機同一 ROS domain 中不能有另一個 publisher 占用同一相機根座標。

要求運作中的 publisher 重讀檔案：

```bash
ros2 service call /easy_handeye2/ur3_realsense_eob/reload_calibration \
  std_srvs/srv/Trigger '{}'
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

成功重讀會更新既有靜態邊，新的 listener 也會收到。若改變 frame 定義或校正類型，需重啟 publisher。發布程序停止後，既有 listener 可能仍保留靜態 TF；可用新啟動的 `tf2_echo` 檢查發布。

## 操作檢查

流程會印出 `~/.ros/log/handeye-workflow-*/` 下的紀錄路徑：`aruco_tracker.log` 與 `calibration_server.log`。偵測不到標記時檢查影像、字典、ID、邊長與可見性；預檢失敗則調整起始姿態再試。服務提供者衝突時需停止重複的校正程序。開始 pick up 前，檢查點雲與手臂模型在手臂座標下是否對齊；發布成功不代表校正精度正確。原生資源清理崩潰是異常退出，應確認儲存訊息與發布的轉換，不可只依程序退出推定成功。

校正檔格式、儲存目錄與 `/easy_handeye2/<name>/reload_calibration` 服務介面皆維持一致；pick_up 只需可用 TF。套件沿用隨附 LGPLv3 授權。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [LICENSE.md](LICENSE.md) | 隨附 LGPLv3 授權條款與原始授權文字。 |
| [README.md](README.md) | 英文操作指南：自動校正、啟動清理、參數、TF 發布及操作檢查。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：自動校正、啟動清理、參數、TF 發布及操作檢查。 |
| [auto_calibration/__init__.py](auto_calibration/__init__.py) | 標示自動校正 Python 套件並提供套件簡介。 |
| [auto_calibration/calibrate.py](auto_calibration/calibrate.py) | 自動校正入口；協調啟動清理、tracker、校正服務、確認提示、移動程序與 TF 發布。 |
| [auto_calibration/motion.py](auto_calibration/motion.py) | 服務 client、15 點姿態生成、規劃診斷與重試、自動移動、樣本清零及計算／儲存流程。 |
| [auto_calibration/publisher.py](auto_calibration/publisher.py) | 讀取校正檔，換算並發布相機根座標 TF，限制單一發布者並提供 reload 服務。 |
| [auto_calibration/startup.py](auto_calibration/startup.py) | 辨識並停止舊校正與同一相機 tracker 程序，檢查 ROS 衝突與資源鎖，阻擋未清理完成的啟動。 |
| [launch/publish.launch.py](launch/publish.launch.py) | 以校正名稱與相機根座標參數，啟動已儲存校正的 TF publisher。 |
| [package.xml](package.xml) | ROS 套件名稱、版本、授權與建置／執行相依宣告。 |
| [resource/auto_calibration](resource/auto_calibration) | ament 索引的套件辨識標記；供 ROS 2 找到套件，空檔案也必須保留。 |
| [setup.cfg](setup.cfg) | 將 auto_calibration 的註冊執行檔安裝到 lib/auto_calibration，供 ros2 run 尋找。 |
| [setup.py](setup.py) | 註冊 calibrate 與 handeye_publisher 指令，安裝 launch、授權與雙語 README。 |
