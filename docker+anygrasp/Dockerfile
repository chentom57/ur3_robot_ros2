# Stage 1: the PyTorch/CUDA combination tested separately on the user's GPU.
FROM pytorch/pytorch:2.5.1-cuda12.1-cudnn9-devel AS base

# RTX 3060 Laptop is Ampere compute capability 8.6.
# docker build does not need access to a GPU: specify architecture explicitly.
ENV CUDA_HOME=/usr/local/cuda \
    TORCH_CUDA_ARCH_LIST=8.6 \
    MAX_JOBS=2 \
    OMP_NUM_THREADS=2

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git ca-certificates libopenblas-dev \
    && rm -rf /var/lib/apt/lists/*

# Keep the older extension build environment separate from system ROS Python.
RUN python -m pip install --no-cache-dir \
      numpy==1.26.4 setuptools==69.5.1 wheel ninja \
    && python -c "import torch; assert torch.version.cuda == '12.1', torch.version.cuda" \
    && nvcc --version

# Stage 2: the CUDA 12 fork linked from the official AnyGrasp installation guide.
FROM base AS minkowski
ARG MINKOWSKI_REF=cuda-12-1
RUN git clone --branch "${MINKOWSKI_REF}" --single-branch \
      https://github.com/chenxi-wang/MinkowskiEngine.git /opt/MinkowskiEngine \
    && git -C /opt/MinkowskiEngine rev-parse HEAD > /opt/minkowski-source-revision.txt
WORKDIR /opt/MinkowskiEngine
RUN python setup.py install --force_cuda --blas=openblas \
      --blas_include_dirs=/usr/include/x86_64-linux-gnu \
      --blas_library_dirs=/usr/lib/x86_64-linux-gnu

WORKDIR /workspace
CMD ["bash"]
