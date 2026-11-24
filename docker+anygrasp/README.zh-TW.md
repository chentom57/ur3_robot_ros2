# Docker 與 AnyGrasp 操作指南

[English](README.md)

本資料夾提供 Ubuntu 24.04 x86_64 上的 Docker 安裝、GPU 支援、容器基本操作，以及 AnyGrasp 的建置、授權與推論流程。適用目前的 RTX 3060 Laptop 6 GB 環境。主機保留 ROS 2 環境，AnyGrasp 在 Docker 裡執行。

## 相依項目與安裝分工

| 相依項目 | 準備方式 |
| --- | --- |
| Ubuntu 24.04 x86_64 與 NVIDIA GPU | 先安裝主機 NVIDIA 驅動並確認 `nvidia-smi`，見第 4 節與其中的官方驅動指南。 |
| Docker Engine | 依第 3 節從 Docker apt 來源安裝。 |
| NVIDIA Container Toolkit | 依第 4 節安裝與設定，再確認容器內可看到 GPU。 |
| PyTorch／CUDA、OpenBLAS 與 MinkowskiEngine | 第 5 節的基底 Dockerfile 在容器內建置；其他 GPU 需調整 CUDA 架構 `8.6`。 |
| AnyGrasp SDK、PointNet2、GraspNetAPI、Open3D 與共用函式庫 | 第 5 節由 `Dockerfile.sdk` 安裝，不需在主機用 pip 安裝。 |
| 授權與 detection 權重 | 依第 6 節用新環境的穩定 feature ID 申請並放置檔案，原始碼不附這些資料。 |
| ROS、相機、TF 與 MoveIt | 即時夾取對接才需要，依[主機建置](../ROS2+driver/ur3_ros2_setup.zh-TW.md)與[夾取安裝](../ur3_pick_up/README.zh-TW.md)準備；官方資料示範不需 ROS 或連接硬體。 |

複製來源的範例以 `project_dir="$HOME/ur3_ws/src/robot_arm"` 指定完整專案位置。若專案放在別處，請修改此變數。模型工作目錄維持 `~/anygrasp_docker`，放在 ROS 原始碼目錄之外。

## 1. 目錄與環境

| 項目 | 位置／設定 |
| --- | --- |
| 本文件與建置檔 | `~/ur3_ws/src/robot_arm/docker+anygrasp` |
| 實際建置／運算工作目錄 | `~/anygrasp_docker` |
| SDK 容器內位置 | `/opt/anygrasp_sdk` |
| 日常使用容器 | `anygrasp-fixed` |
| GPU | NVIDIA RTX 3060 Laptop，6 GB |
| 模型環境 | PyTorch 2.5.1 + CUDA 12.1、Python 3.11、MinkowskiEngine 0.5.4 |
| 機器人環境 | 主機 ROS 2 Jazzy，與模型 Python 分開 |

檔案：[Dockerfile](Dockerfile) 建置 PyTorch／MinkowskiEngine 基底；[Dockerfile.sdk](Dockerfile.sdk) 安裝 SDK 相依套件；[demo_once.py](demo_once.py) 只對官方範例資料執行一次推論。授權檔、權重、推論結果放在工作目錄或容器，不放入桌面原始碼資料夾。

本文中「主機」指 `chentom@...` 終端，「容器」指 `root@anygrasp-fixed:...`。Docker 指令在主機執行；進入容器後，通常不需再加 sudo。所有安裝指令都是手動操作步驟，不是本文件自動執行的動作。

## 2. 已有環境：日常啟動與推論

已具備 `anygrasp-fixed`、有效 license 與 detection 權重時，直接使用本節。模型單次範例推論已在本機環境通過，產生 98 個候選；候選數與評分可能隨環境／資料不同而變化。

**主機終端：**

```bash
sudo docker ps -a
sudo docker start anygrasp-fixed
sudo docker cp "$HOME/anygrasp_docker/demo_once.py" \
  anygrasp-fixed:/opt/anygrasp_sdk/grasp_detection/demo_once.py
sudo docker exec -it -w /opt/anygrasp_sdk/grasp_detection \
  anygrasp-fixed python demo_once.py \
  --checkpoint_path log/checkpoint_detection.tar \
  --max_gripper_width 0.085
```

若工作目錄還沒有 demo_once.py，從本資料夾複製：

```bash
project_dir="$HOME/ur3_ws/src/robot_arm"
cp "$project_dir/docker+anygrasp/demo_once.py" \
  "$HOME/anygrasp_docker/demo_once.py"
```

成功訊息為 `Single inference passed: ... candidates`。輸出在容器的 `/opt/anygrasp_sdk/grasp_detection/results/official_demo_grasps.npy`。

複製結果回主機：

```bash
mkdir -p "$HOME/anygrasp_docker/exchange"
sudo docker cp \
  anygrasp-fixed:/opt/anygrasp_sdk/grasp_detection/results/official_demo_grasps.npy \
  "$HOME/anygrasp_docker/exchange/official_demo_grasps.npy"
```

需要互動 shell 時：

```bash
sudo docker exec -it -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed bash
```

在這個 exec shell 輸入 `exit` 只會離開該 shell，主容器繼續運作。停止整個容器用主機的 `sudo docker stop anygrasp-fixed`。接下來第 3～7 節供首次建立環境使用，已能推論就不需重建。

## 3. 主機安裝 Docker Engine

適用 Ubuntu 24.04 amd64。先確認 `cat /etc/os-release` 與 `uname -m`。若 `sudo docker run --rm hello-world` 已成功，可跳到 GPU 設定。

本流程使用 Docker 官方 apt 套件庫；若主機已有 `docker.io`、`podman-docker` 等其他來源的引擎，先依[官方相衝突套件說明](https://docs.docker.com/engine/install/ubuntu/#uninstall-old-versions)處理，再安裝。不要因為閱讀本指南而刪除目前能運作的容器。

**以下全部在主機執行：**

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<'EOF'
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: noble
Components: stable
Architectures: amd64
Signed-By: /etc/apt/keyrings/docker.asc
EOF
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin
sudo systemctl enable --now docker
sudo docker run --rm hello-world
```

`Hello from Docker!` 表示基本容器執行成功。本指南統一使用 sudo，不需為了模型安裝更改 Docker socket 權限。[Docker 安裝來源](https://docs.docker.com/engine/install/ubuntu/)

## 4. 啟用容器 GPU

先在主機確認 NVIDIA 驅動：

```bash
nvidia-smi
```

若主機就無法辨識 GPU，先完成[適用 Linux 的 NVIDIA 驅動安裝](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/)。驅動位於主機；CUDA toolkit、PyTorch 與編譯工具由模型映像提供。

在主機加入 NVIDIA Container Toolkit 套件庫並安裝：

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor --yes \
      -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
sudo docker run --rm --runtime=nvidia --gpus all ubuntu:24.04 nvidia-smi
```

最後一步應在容器內列出同一張 GPU。重啟 Docker 服務可能中斷正在運作的容器，此步屬首次 GPU 設定。[NVIDIA 官方安裝／設定](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

`nvidia-smi` 的 CUDA Version 是驅動可支援的 CUDA 上限；實際 PyTorch 使用的 CUDA 版本要看 `torch.version.cuda`。本建置使用 CUDA 12.1，不需照驅動畫面的最高版本重新選 PyTorch。

## 5. 在 Docker 建置 AnyGrasp 環境

### 準備工作目錄

**主機：**

```bash
project_dir="$HOME/ur3_ws/src/robot_arm"
mkdir -p "$HOME/anygrasp_docker"
cp "$project_dir/docker+anygrasp/Dockerfile" \
  "$HOME/anygrasp_docker/Dockerfile"
cp "$project_dir/docker+anygrasp/Dockerfile.sdk" \
  "$HOME/anygrasp_docker/Dockerfile.sdk"
cp "$project_dir/docker+anygrasp/.dockerignore" \
  "$HOME/anygrasp_docker/.dockerignore"
cp "$project_dir/docker+anygrasp/demo_once.py" \
  "$HOME/anygrasp_docker/demo_once.py"
cd "$HOME/anygrasp_docker"
```

### 建置 MinkowskiEngine

```bash
sudo docker build --progress=plain --target minkowski -t anygrasp:minkowski .
```

- `FROM pytorch/pytorch:2.5.1-cuda12.1-cudnn9-devel`：使用含 PyTorch、CUDA 編譯工具的基底。
- 使用官方連結的 `chenxi-wang/MinkowskiEngine`，分支 `cuda-12-1`；不是直接使用另一個未修改版本。[官方 SDK 安裝](https://github.com/graspnet/anygrasp_sdk#installation)
- `TORCH_CUDA_ARCH_LIST=8.6` 對應此 RTX 3060，換 GPU 架構需調整；`--force_cuda` 讓建置階段不必存取實際 GPU。
- `MAX_JOBS=2` 限制編譯平行度；NumPy 1.26.4／setuptools 69.5.1 維持擴充模組的建置環境。
- `CUDA_HOME`、OpenBLAS 路徑是容器內的設定。OpenBLAS 使用 apt 安裝，因此不用照抄官方 conda 安裝命令。

驗證 GPU 稀疏卷積：

```bash
sudo docker run --rm --runtime=nvidia --gpus all anygrasp:minkowski python -c '
import torch
import MinkowskiEngine as ME
coords = torch.tensor([[0,0,0,0], [0,1,0,0]], dtype=torch.int32)
features = torch.ones((2,3), dtype=torch.float32)
x = ME.SparseTensor(features=features, coordinates=coords, device="cuda")
layer = ME.MinkowskiConvolution(3,4,kernel_size=1,dimension=3).cuda()
y = layer(x)
assert y.F.is_cuda
print("MinkowskiEngine GPU test passed:", y.F.shape, y.F.device)
'
```

### 建置 SDK 相依套件

```bash
sudo docker build --progress=plain -f Dockerfile.sdk -t anygrasp:sdk-deps .
```

此映像包含 SDK 原始碼、PointNet2、graspnetAPI、Open3D 所需系統函式庫與 `net-tools`。`net-tools` 提供授權機器碼收集會使用的 ifconfig。License 與模型權重在容器建立後另外放入，不寫入 Dockerfile。

驗證模組與 CUDA：

```bash
sudo docker run --rm --runtime=nvidia --gpus all anygrasp:sdk-deps python -c '
import torch
import MinkowskiEngine as ME
import pointnet2._ext
from graspnetAPI import GraspGroup
print("PyTorch:", torch.__version__)
print("CUDA:", torch.version.cuda)
print("MinkowskiEngine:", ME.__version__)
assert torch.cuda.is_available()
assert len(GraspGroup()) == 0
print("SDK dependencies import passed")
'
```

建置會把各套件 Git commit 記錄於 `/opt/*source-revision.txt`。建置耗時主要在 CUDA 擴充編譯；修改後重新 build 可利用既有層快取，不需每次移除映像。

## 6. 建立固定容器與驗證 license

### 首次建立容器

已有 `anygrasp-fixed` 就使用第 2 節，不要再次執行同名 run。以下為首次建立：

```bash
sudo docker run -dit \
  --name anygrasp-fixed \
  --hostname anygrasp-fixed \
  --mac-address 22:d7:05:d4:f7:fe \
  --runtime=nvidia --gpus all \
  anygrasp:sdk-deps bash
```

本流程使用 Docker bridge 網路，固定容器名稱、hostname 與 MAC。不要用 `--rm` 建立授權工作容器；退出／停止後需保留容器資料。固定 MAC 是此工作流程維持指紋的設定，不能據此保證在不同電腦上取得相同機器碼或沿用 license。

### 取得機器碼

**主機：**

```bash
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  cp ../grasp_detection/gsnet_versions/gsnet.cpython-311-x86_64-linux-gnu.so gsnet.so
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  python -c 'from gsnet import get_feature_id; print(get_feature_id())'
```

Python 3.11 對應 `cpython-311`；換 Python 版本需更換二進位檔。在申請前重啟容器並重查一次機器碼：

```bash
sudo docker restart anygrasp-fixed
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  python -c 'from gsnet import get_feature_id; print(get_feature_id())'
```

確認機器碼固定後，依[官方授權申請說明](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md)填寫表單。目前本機已核對的機器碼為 `N85678130621451655547`，且已通過 license 驗證。新的環境必須使用其實際取得的機器碼，不能直接照抄此數字。

### 放入 license 與模型權重

將收到的授權 ZIP 解壓，使主機目錄結構為：

```text
~/anygrasp_docker/assets/license/
  licenseCfg.json
  <name>.lic
  <name>.public_key
  <name>.signature
```

假設 detection 權重已下載為 `~/Downloads/checkpoint_detection.tar`：

```bash
sudo docker exec anygrasp-fixed mkdir -p \
  /opt/anygrasp_sdk/grasp_detection/license \
  /opt/anygrasp_sdk/grasp_detection/log
sudo docker cp "$HOME/anygrasp_docker/assets/license/." \
  anygrasp-fixed:/opt/anygrasp_sdk/grasp_detection/license/
sudo docker cp "$HOME/Downloads/checkpoint_detection.tar" \
  anygrasp-fixed:/opt/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar
sudo docker exec -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed \
  cp gsnet_versions/gsnet.cpython-311-x86_64-linux-gnu.so gsnet.so
sudo docker exec -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed \
  python -c 'from gsnet import get_feature_id, check_license; print(get_feature_id()); check_license("license")'
```

成功為 `License passed: True, state: PASSED`。`checkpoint_detection.tar` 是供 PyTorch 讀取的 checkpoint，不需用 tar 解壓；tracking 權重供另一個追蹤任務使用。模型權重使用授權回信提供的下載連結。[SDK 檔案放置要求](https://github.com/graspnet/anygrasp_sdk/tree/main/grasp_detection)

## 7. 執行模型與理解輸出

推薦使用第 2 節的 `demo_once.py`。它載入 SDK 自己的函式，使用一次 `dense_grasp=False`、`collision_detection=True` 推論，並輸出 `N×17` GraspGroup 陣列。官方 `demo.py` 會依序執行多種設定，完整流程在 6 GB GPU 上可能耗盡記憶體。[官方 demo](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/demo.py)

| demo_once.py 參數 | 預設值 | 用途 |
| --- | --- | --- |
| `--sdk-dir` | 程式所在目錄 | 容器內 grasp_detection 目錄。 |
| `--checkpoint_path` | `log/checkpoint_detection.tar` | 模型權重；相對路徑以 sdk-dir 為準。 |
| `--max_gripper_width` | `0.085` | 允許的最大夾爪開口，公尺，(0, 0.1]。 |
| `--gripper_height` | `0.03` | SDK 夾指高度設定，公尺；使用實體時須符合工具模型。 |
| `--output` | `results/official_demo_grasps.npy` | 候選輸出路徑，相對 sdk-dir。 |

需要執行完整官方 demo 時，可在主機呼叫：

```bash
sudo docker exec -it -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed \
  python demo.py --checkpoint_path log/checkpoint_detection.tar --max_gripper_width 0.085
```

此處未啟用 `--vis`；Open3D 視窗另需設定容器圖形顯示，不在本流程內。

範例讀取 SDK 的 `example_data`，由單視角 RGB-D 與範例內參產生點雲。N 個候選表示 N 種夾取姿態，不是 N 個物體；結果屬於官方場景，不能作為目前手臂的實體目標。AnyGrasp 接收點雲估計夾取方式，不輸出物體名稱；SAM 等目標選取／分割功能屬於另外的整合。[AnyGrasp 輸入輸出定義](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)

## 8. Docker 基本概念與操作

| 概念 | 在本專案的意義 |
| --- | --- |
| Dockerfile | 安裝步驟清單。build 讀取它。 |
| Image（映像） | 含系統與模型相依套件的範本，例如 `anygrasp:sdk-deps`。 |
| Container（容器） | 由映像建立的執行環境，例如 `anygrasp-fixed`，具有自己的可寫資料層。 |
| 主機檔案 | `~/anygrasp_docker` 等一般目錄，不會自動出現在容器。 |
| docker cp | 在主機與容器之間複製檔案；後續修改不會自動同步。 |
| Bind mount | 建立容器時將主機目錄掛入容器，兩邊看到同一份資料。 |

| 主機指令 | 意義 |
| --- | --- |
| `sudo docker images` | 查看映像。 |
| `sudo docker ps` / `sudo docker ps -a` | 查看執行中／全部容器。 |
| `sudo docker start anygrasp-fixed` | 啟動已存在的容器，不建立新容器。 |
| `sudo docker start -ai anygrasp-fixed` | 啟動並接到主程序；bash 主程序 exit 會讓容器停止。 |
| `sudo docker exec -it anygrasp-fixed bash` | 在運作中的容器開另一個 shell。 |
| `sudo docker stop anygrasp-fixed` | 停止容器，保留資料。 |
| `sudo docker logs --tail 100 anygrasp-fixed` | 查看主程序紀錄；exec 程序的輸出通常看其終端。 |
| `sudo docker exec anygrasp-fixed nvidia-smi` | 查看容器可見的 GPU。 |

`docker build --progress=plain --target minkowski -t anygrasp:minkowski .` 中，`--progress=plain` 顯示文字建置紀錄，`--target` 選擇 Dockerfile 階段，`-t` 指定映像名稱，最後的 `.` 是建置上下文目錄。這個指令建立映像，不會進入互動 shell。

`docker run` 建立新容器；`--name` 命名，`--hostname` 設定容器主機名，`--mac-address` 指定網卡 MAC，`--gpus all` 提供 GPU，`--runtime=nvidia` 使用已設定的 NVIDIA runtime，`-i` 保持標準輸入、`-t` 配置終端、`-d` 背景執行。`--rm` 在容器結束時刪除它，只用於臨時測試。[run](https://docs.docker.com/reference/cli/docker/container/run/)、[exec](https://docs.docker.com/reference/cli/docker/container/exec/)

容器停止不會刪除其中資料，但 `docker rm` 會刪除容器可寫層。重建映像不會自動更新已存在的容器。`docker commit` 可將可寫層存成另一個映像，但不包含 bind mount／volume 內的資料；推論環境以 Dockerfile 維護，license 與權重另外保存。現有授權容器不需為了閱讀文件重建。

本流程以 `docker cp` 傳遞檔案，不需另外建立 bind mount。

## 9. 常見問題

| 情況 | 處理 |
| --- | --- |
| `permission denied ... docker.sock` | 在主機使用本指南的 sudo docker 指令。 |
| 容器未運作，exec 失敗 | 先 docker ps -a，再 docker start anygrasp-fixed。 |
| 同名容器已存在 | 使用 start／exec，不要反覆 run。 |
| `failed to collect MAC addresses from ifconfig` | 映像需安裝 net-tools；本資料夾 Dockerfile.sdk 已包含。檢查二進位 Python 版本與容器網路。 |
| 重啟後機器碼改變 | 檢查是否使用原容器、固定 MAC 與原網路配置，先重查 get_feature_id。 |
| License 失敗 | 核對機器碼、license/ 四個檔案及 Python 對應的 gsnet.so。 |
| `libEGL.so.1`／`libusb-1.0.so.0` 缺失 | 本 Dockerfile.sdk 安裝 libegl1、libusb-1.0-0。 |
| grasp_nms metadata 缺 Cython | 本 Dockerfile.sdk 在安裝 graspnetAPI 前安裝 Cython<3。 |
| 找不到 pointnet2._ext | 在 /workspace 驗證安裝，避免在原始碼 pointnet2 目錄測試時遮蔽套件。 |
| CUDA out of memory | 先用 demo_once.py；確認沒有其他程序占用 GPU，再縮小實際輸入規模。重新執行是新的 Python 程序。 |
| ROS／catkin import warnings | 模型容器未安裝 ROS；範例推論不使用這些 ROS 輔助方法。 |
| torch.load FutureWarning | 本版本權重載入的提示，不是該次 OOM 的錯誤來源。 |
| `groups: cannot find name for group ID 992` | 群組名稱查詢提示；若推論已正常進行，無須為此重裝環境。 |

## 10. 官方參考

- [Docker Engine：Ubuntu 安裝](https://docs.docker.com/engine/install/ubuntu/)
- [Docker run](https://docs.docker.com/reference/cli/docker/container/run/) 與 [exec](https://docs.docker.com/reference/cli/docker/container/exec/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [AnyGrasp SDK](https://github.com/graspnet/anygrasp_sdk)
- [License 申請與驗證](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md)
- [Detection 檔案配置](https://github.com/graspnet/anygrasp_sdk/tree/main/grasp_detection) 與 [API](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)

## 即時相機與 pick_up 對接

主機的 `ur3_pick_up` 會自動取得新 PointCloud2，將人工圈選影像區域映射至點雲，傳入既有 `anygrasp-fixed` 推論，依夾爪開口篩選後按分數嘗試候選，直到去程及返回路徑都規劃成功。SDK、license 與權重維持在容器；不需把 ROS 安裝進容器。每次傳入 worker／點雲並取回結果，不需重新建立容器或更換 MAC。

先啟動 bringup 與唯一手眼 publisher，然後在主機同一終端：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

只推論與規劃、完全不移動：加 `--ros-args -p plan_only:=true`。完整參數及執行確認流程見 [pick_up 操作說明](../ur3_pick_up/README.zh-TW.md)。推論紀錄保留於 `~/anygrasp_docker/exchange/`。

<a id="file-inventory"></a>
## 檔案名稱與用途

下表逐一列出本資料夾及子資料夾的原始檔，路徑相對於目前資料夾；不含建置結果、執行紀錄與快取。

| 檔案名稱 | 功用 |
| --- | --- |
| [.dockerignore](.dockerignore) | 排除授權、權重封存檔、結果與快取等不需送入 Docker build 的內容。 |
| [.gitignore](.gitignore) | 排除本機 SDK 素材、授權、權重封存檔、推論結果與快取，避免提交。 |
| [Dockerfile](Dockerfile) | 以 PyTorch／CUDA 開發映像建立基底，設定 GPU 架構並編譯官方指定分支的 MinkowskiEngine。 |
| [Dockerfile.sdk](Dockerfile.sdk) | 在 MinkowskiEngine 映像上安裝 SDK、PointNet2、GraspNetAPI、Open3D 相依與必要系統函式庫。 |
| [README.md](README.md) | 英文操作指南：Docker／GPU 安裝、SDK 建置、授權與權重準備、推論及容器操作。 |
| [README.zh-TW.md](README.zh-TW.md) | 繁體中文操作指南：Docker／GPU 安裝、SDK 建置、授權與權重準備、推論及容器操作。 |
| [demo_once.py](demo_once.py) | 只對 SDK 官方範例場景推論一次並儲存候選，供模型環境驗證，不使用即時相機。 |
