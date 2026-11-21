# Docker and AnyGrasp guide

[繁體中文](README.zh-TW.md)

This guide covers Docker Engine installation, NVIDIA GPU access, everyday container operations, and AnyGrasp environment building, licensing and inference on Ubuntu 24.04 x86_64 with an RTX 3060 Laptop 6 GB GPU. ROS 2 stays on the host; model inference runs in Docker.

## Prerequisites and installation responsibilities

| Dependency | Preparation |
| --- | --- |
| Ubuntu 24.04 x86_64 and NVIDIA GPU | Install the host NVIDIA driver and verify `nvidia-smi`; see Section 4 and its official driver guide. |
| Docker Engine | Install from Docker's apt repository in Section 3. |
| NVIDIA Container Toolkit | Install/configure in Section 4, then confirm the GPU is visible inside Docker. |
| PyTorch/CUDA, OpenBLAS and MinkowskiEngine | Built inside the base Dockerfile in Section 5. Adjust CUDA architecture `8.6` for a different GPU. |
| AnyGrasp SDK, PointNet2, GraspNetAPI, Open3D and shared libraries | Installed inside `Dockerfile.sdk` in Section 5; no host pip installation is required. |
| License and detection weights | Apply with the new environment's stable feature ID and place files as shown in Section 6. These files are not in the repository. |
| ROS, camera, TF and MoveIt | Needed only for live pick-up integration; follow [host setup](../ROS2+driver/ur3_ros2_setup.md) and [pick-up installation](../ur3_pick_up/README.md). The official-data demo does not require ROS or connected hardware. |

Source-copy examples use `project_dir="$HOME/ur3_ws/src/robot_arm"`, the complete-repository layout. Change this variable if the repository is elsewhere. Keep the model working directory at `~/anygrasp_docker`, outside the ROS source tree.

## 1. Files and environment

| Item | Location / configuration |
| --- | --- |
| Documentation and build recipes | `~/ur3_ws/src/robot_arm/docker+anygrasp` |
| Working/build directory | `~/anygrasp_docker` |
| SDK inside container | `/opt/anygrasp_sdk` |
| Persistent working container | `anygrasp-fixed` |
| Model environment | PyTorch 2.5.1 + CUDA 12.1, Python 3.11, MinkowskiEngine 0.5.4 |
| Robot environment | Host ROS 2 Jazzy, separate Python environment |

[Dockerfile](Dockerfile) builds the PyTorch/MinkowskiEngine base. [Dockerfile.sdk](Dockerfile.sdk) adds the SDK dependencies. [demo_once.py](demo_once.py) runs a single inference on the SDK example scene. Keep private licenses, weights and generated outputs in the working directory/container rather than this source folder.

“Host” means the `chentom@...` terminal. “Container” means `root@anygrasp-fixed:...`. Run Docker commands on the host; commands inside the container usually need no sudo. The installation commands below are manual instructions, not actions automatically performed by this document.

## 2. Existing installation: daily inference

If `anygrasp-fixed` already has a valid license and detection checkpoint, use this section. Single inference on this machine has produced 98 candidates; counts and scores can vary.

**Host terminal:**

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

If the working directory lacks the helper, copy it from this folder:

```bash
project_dir="$HOME/ur3_ws/src/robot_arm"
cp "$project_dir/docker+anygrasp/demo_once.py" \
  "$HOME/anygrasp_docker/demo_once.py"
```

Success prints `Single inference passed: ... candidates`. The file is stored at `/opt/anygrasp_sdk/grasp_detection/results/official_demo_grasps.npy` inside the container.

Copy the output to the host:

```bash
mkdir -p "$HOME/anygrasp_docker/exchange"
sudo docker cp \
  anygrasp-fixed:/opt/anygrasp_sdk/grasp_detection/results/official_demo_grasps.npy \
  "$HOME/anygrasp_docker/exchange/official_demo_grasps.npy"
```

Open an interactive shell when needed:

```bash
sudo docker exec -it -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed bash
```

`exit` in this exec shell closes that shell; the main container keeps running. Use `sudo docker stop anygrasp-fixed` on the host to stop the container. Sections 3–7 describe first-time setup; a working installation does not need rebuilding.

## 3. Install Docker Engine on the host

The following apt repository configuration targets Ubuntu 24.04 amd64 (`noble`). Check `cat /etc/os-release` and `uname -m`. If `sudo docker run --rm hello-world` already works, continue to GPU configuration.

This uses Docker's official repository. If unofficial/conflicting engine packages such as docker.io or podman-docker are installed, follow the [official conflict-removal instructions](https://docs.docker.com/engine/install/ubuntu/#uninstall-old-versions) before installing. Do not remove a working licensed container merely to follow the guide.

**Host commands:**

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

`Hello from Docker!` confirms basic container execution. Commands in this guide use sudo; changing socket permissions is not required. [Docker installation reference](https://docs.docker.com/engine/install/ubuntu/)

## 4. Enable GPU access

Check the host GPU driver first:

```bash
nvidia-smi
```

If this fails, complete the [NVIDIA Linux driver installation](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/) first. The driver runs on the host; the image supplies CUDA development tools and PyTorch.

Install and configure NVIDIA Container Toolkit on the host:

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

The final command should list the GPU inside a container. Restarting Docker can interrupt existing containers; this is a first-time GPU configuration step. [NVIDIA installation and configuration](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

The CUDA version displayed by nvidia-smi is the driver's supported ceiling. Check `torch.version.cuda` for the runtime used by PyTorch. This recipe uses CUDA 12.1; do not select a different PyTorch build solely to match nvidia-smi's maximum.

## 5. Build the AnyGrasp environment

Prepare the host working directory:

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

Build the MinkowskiEngine image:

```bash
sudo docker build --progress=plain --target minkowski -t anygrasp:minkowski .
```

The base is `pytorch/pytorch:2.5.1-cuda12.1-cudnn9-devel`. MinkowskiEngine comes from the fork linked by the [SDK installation guide](https://github.com/graspnet/anygrasp_sdk#installation), using branch `cuda-12-1`. `TORCH_CUDA_ARCH_LIST=8.6` targets this RTX 3060; change it for another GPU architecture. `--force_cuda` enables CUDA compilation without exposing a physical GPU during build. `MAX_JOBS=2` limits parallel compilation. NumPy 1.26.4 and setuptools 69.5.1 keep the extension build environment consistent.

CUDA_HOME and OpenBLAS paths are inside the container. OpenBLAS is installed with apt, so the Dockerfile does not reproduce the official conda commands verbatim.

Verify sparse CUDA convolution:

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

Build SDK dependencies:

```bash
sudo docker build --progress=plain -f Dockerfile.sdk -t anygrasp:sdk-deps .
```

This image includes SDK sources, PointNet2, graspnetAPI, Open3D system libraries and net-tools (providing ifconfig for feature-ID collection). Copy the license and weights after container creation; they are not embedded in the Dockerfile.

Verify imports and CUDA:

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

Source revisions are recorded in `/opt/*source-revision.txt`. CUDA extension compilation accounts for much of the build time. Rebuilding can reuse layer cache; deleting images before every build is unnecessary.

## 6. Create the fixed container and validate the license

If `anygrasp-fixed` already exists, use start/exec from Section 2. Run this only to create it for the first time:

```bash
sudo docker run -dit \
  --name anygrasp-fixed \
  --hostname anygrasp-fixed \
  --mac-address 22:d7:05:d4:f7:fe \
  --runtime=nvidia --gpus all \
  anygrasp:sdk-deps bash
```

The container uses Docker bridge networking with a fixed name, hostname and MAC. Do not use --rm for this persistent licensed environment. A fixed MAC is part of this workflow's fingerprint configuration; it does not guarantee a matching feature ID or transferable license on a different computer.

Collect the feature ID from the host:

```bash
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  cp ../grasp_detection/gsnet_versions/gsnet.cpython-311-x86_64-linux-gnu.so gsnet.so
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  python -c 'from gsnet import get_feature_id; print(get_feature_id())'
```

The `cpython-311` binary matches Python 3.11. Select another SDK binary if the Python version differs. Before applying, restart and verify that the feature ID remains unchanged:

```bash
sudo docker restart anygrasp-fixed
sudo docker exec -w /opt/anygrasp_sdk/license_registration anygrasp-fixed \
  python -c 'from gsnet import get_feature_id; print(get_feature_id())'
```

Apply through the [official license registration instructions](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md). This machine's verified feature ID is `N85678130621451655547`, and its license has passed. A new environment must use its own measured ID, not copy this number.

Extract the received ZIP into this host layout:

```text
~/anygrasp_docker/assets/license/
  licenseCfg.json
  <name>.lic
  <name>.public_key
  <name>.signature
```

Assuming the detection checkpoint is downloaded to `~/Downloads/checkpoint_detection.tar`, copy and validate:

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

Success prints `License passed: True, state: PASSED`. The checkpoint is read by PyTorch directly; do not unpack it with tar. Tracking weights serve a separate tracking task. Use the model download links supplied in the license email. [SDK file placement](https://github.com/graspnet/anygrasp_sdk/tree/main/grasp_detection)

## 7. Inference and output

Use demo_once.py as shown in Section 2 for a single non-dense inference with collision filtering. It calls the SDK's own helper functions and exports an N×17 GraspGroup array. The [official demo](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/demo.py) runs several configurations sequentially; the full sequence can exhaust a 6 GB GPU.

| demo_once.py option | Default | Meaning |
| --- | --- | --- |
| `--sdk-dir` | Script directory | Container grasp_detection directory. |
| `--checkpoint_path` | `log/checkpoint_detection.tar` | Model checkpoint, relative to sdk-dir when not absolute. |
| `--max_gripper_width` | `0.085` | Maximum opening in meters, (0, 0.1]. |
| `--gripper_height` | `0.03` | SDK finger-height setting in meters; match the tool model for real use. |
| `--output` | `results/official_demo_grasps.npy` | Output path relative to sdk-dir. |

To run the full SDK demo from the host:

```bash
sudo docker exec -it -w /opt/anygrasp_sdk/grasp_detection anygrasp-fixed \
  python demo.py --checkpoint_path log/checkpoint_detection.tar --max_gripper_width 0.085
```

This command omits --vis; Open3D windows need additional container display configuration, outside this procedure.

The sample uses single-view RGB-D data and example camera intrinsics from example_data. N candidates mean N grasp poses, not N objects. These poses belong to the example scene and are not real targets for the current robot. AnyGrasp predicts grasps from point clouds; object names and SAM-based target selection are separate functionality. [SDK API and format](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)

## 8. Docker concepts and basic operations

| Concept | Meaning here |
| --- | --- |
| Dockerfile | Build instructions for an image. |
| Image | Environment template such as anygrasp:sdk-deps. |
| Container | Runtime instance such as anygrasp-fixed, with its own writable filesystem layer. |
| Host files | Ordinary paths such as ~/anygrasp_docker; not automatically visible in containers. |
| docker cp | Copies files; subsequent edits are not automatically synchronized. |
| Bind mount | A host directory exposed in the container, sharing the same data. |

| Host command | Purpose |
| --- | --- |
| `sudo docker images` | List images. |
| `sudo docker ps` / `sudo docker ps -a` | List running / all containers. |
| `sudo docker start anygrasp-fixed` | Start an existing container. |
| `sudo docker start -ai anygrasp-fixed` | Start and attach to the main process; exiting its main bash stops the container. |
| `sudo docker exec -it anygrasp-fixed bash` | Open another shell in a running container. |
| `sudo docker stop anygrasp-fixed` | Stop, keeping container data. |
| `sudo docker logs --tail 100 anygrasp-fixed` | View main-process logs; exec output is normally read in its terminal. |
| `sudo docker exec anygrasp-fixed nvidia-smi` | Check the GPU visible inside the container. |

For `docker build --progress=plain --target minkowski -t anygrasp:minkowski .`: --progress=plain prints text logs; --target selects a Dockerfile stage; -t names the image; the final dot is the build-context directory. Build creates an image, not an interactive shell.

Docker run creates a new container. --name names it; --hostname sets its hostname; --mac-address sets a MAC; --gpus all exposes GPUs; --runtime=nvidia chooses the configured NVIDIA runtime. -i keeps stdin open, -t allocates a terminal, -d detaches into the background. --rm deletes a container after it exits and is used here only for disposable tests. [run](https://docs.docker.com/reference/cli/docker/container/run/) / [exec](https://docs.docker.com/reference/cli/docker/container/exec/)

Stopping preserves container files; docker rm deletes its writable layer. Rebuilding an image does not update existing containers. Docker commit can capture a writable layer but not bind-mount/volume contents. Maintain software through Dockerfiles and store licenses/weights separately; a working licensed container does not need replacing just to read this guide.

This workflow transfers files with `docker cp`; no bind-mount setup is required.

## 9. Troubleshooting

| Symptom | Check |
| --- | --- |
| docker.sock permission denied | Use the host sudo docker commands. |
| exec says container is not running | docker ps -a, then docker start anygrasp-fixed. |
| Container name already in use | Use start/exec rather than another run. |
| MAC collection fails through ifconfig | Ensure net-tools is installed; this Dockerfile.sdk includes it. Check Python binary and networking. |
| Feature ID changes on restart | Check container identity, fixed MAC and original network configuration; rerun get_feature_id. |
| License rejected | Check feature ID, all four license files and the Python-matching gsnet.so. |
| Missing libEGL / libusb | Dockerfile.sdk installs libegl1 and libusb-1.0-0. |
| grasp_nms metadata lacks Cython | Dockerfile.sdk installs Cython<3 before graspnetAPI. |
| pointnet2._ext unavailable | Test from /workspace to avoid shadowing the installed module with source files. |
| CUDA out of memory | Use demo_once.py first; check other GPU processes and reduce real input size if needed. Each rerun starts a fresh Python process. |
| ROS/catkin import warnings | ROS helper methods are unavailable in this model-only container; the demo does not need them. |
| torch.load FutureWarning | A checkpoint-loading notice, not the reported OOM failure. |
| Group ID 992 has no name | A name-lookup message; it does not require reinstalling an otherwise working inference environment. |

## 10. Official references

- [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
- [AnyGrasp SDK installation](https://github.com/graspnet/anygrasp_sdk)
- [License registration](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md)
- [Detection setup](https://github.com/graspnet/anygrasp_sdk/tree/main/grasp_detection) and [API](https://github.com/graspnet/anygrasp_sdk/blob/main/grasp_detection/USAGE.md)

## Live camera integration with pick_up

The host `ur3_pick_up` package captures a new PointCloud2, requests inference in the existing `anygrasp-fixed` container, and tries valid poses by descending score within a base_link +Y workspace and selects the first pose with successful outbound and home plans. SDK dependencies, license and weights stay inside Docker. ROS is not required inside the container. Per-request worker/cloud files are copied in and results copied out, without recreating the container or changing its MAC.

Start bringup and one calibration publisher first, then run in one host terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ur3_ws/install/setup.bash
sudo -v
ros2 run ur3_pick_up pick_up
```

Append `--ros-args -p plan_only:=true` for inference/planning without any motion. Default bounds are X=[−0.30,+0.30], Y=[0,+0.60], Z=[0.02,0.60] m, displayed in RViz. See the [pick_up guide](../ur3_pick_up/README.md) for parameters and execution confirmations. Request logs are retained in `~/anygrasp_docker/exchange/`.

<a id="file-inventory"></a>
## File inventory

Every source file in this folder and its subfolders is listed below, relative to this folder. Build outputs, runtime logs and caches are excluded.

| File | Purpose |
| --- | --- |
| [.dockerignore](.dockerignore) | Exclude licenses, weight archives, results and caches from the Docker build context. |
| [.gitignore](.gitignore) | Keep local SDK assets, licenses, weight archives, inference outputs and caches out of Git. |
| [Dockerfile](Dockerfile) | Build the PyTorch/CUDA base, configure GPU architecture and compile the specified MinkowskiEngine fork/branch. |
| [Dockerfile.sdk](Dockerfile.sdk) | Extend the MinkowskiEngine image with SDK, PointNet2, GraspNetAPI, Open3D dependencies and required system libraries. |
| [README.md](README.md) | English guide: Docker/GPU installation, SDK build, licensing/weights, inference and container operations. |
| [README.zh-TW.md](README.zh-TW.md) | Traditional Chinese guide: Docker/GPU installation, SDK build, licensing/weights, inference and container operations. |
| [demo_once.py](demo_once.py) | Infer once on the SDK example scene and save candidates for environment validation; does not use the live camera. |
