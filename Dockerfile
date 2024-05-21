# src: cpu, cpu-torch, cuda-torch, rocm-torch, from-source-torch
ARG src=cpu

FROM --platform=$BUILDPLATFORM ubuntu:jammy AS cpu-base
FROM --platform=$BUILDPLATFORM ubuntu:jammy AS cpu-torch-base
FROM --platform=$BUILDPLATFORM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04 AS cuda-torch-base
FROM --platform=$BUILDPLATFORM rocm/dev-ubuntu-22.04:6.0-complete AS rocm-torch-base

FROM ${src}-base AS image

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive \
       apt-get -y --quiet --no-install-recommends install \
	   software-properties-common \
       gpg-agent \
       ca-certificates \
       gpg \
       wget \
    && add-apt-repository ppa:ecal/ecal-latest \
    && test -f /usr/share/doc/kitware-archive-keyring/copyright || wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2> /dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg > /dev/null \
    && echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | tee /etc/apt/sources.list.d/kitware.list > /dev/null \
    && apt-get update \
    && test -f /usr/share/doc/kitware-archive-keyring/copyright || rm /usr/share/keyrings/kitware-archive-keyring.gpg \
    && DEBIAN_FRONTEND=noninteractive \
       apt-get -y --quiet --no-install-recommends install \
       ecal \
       kitware-archive-keyring \
       cmake \
       build-essential \
       pkg-config \
       libprotobuf-dev \
       protobuf-compiler \
       python3-msgpack \
       libmsgpack-dev \
       python3 \
       python3-pip \
       python3-ecal5 \
       python-is-python3 \
       python-dev-is-python3 \
       git \
       gdb \
       libopencv-dev \
       python3-opencv \
       unzip \
       libeigen3-dev \
       nlohmann-json3-dev \
       libomp-dev \
       libtbb-dev \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -fr /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# 1. Line enables networking
# 2. Line enables zero copy mode
# 3. Line enables multi-bufferfing mode, to relax the publisher needs to wait for the subscriber to release the buffer.
# currently 5 buffer, number of buffers can be increased, but need more RAM
RUN awk -F"=" '/^network_enabled/{$2="= true"}1' /etc/ecal/ecal.ini | \
    awk -F"=" '/^memfile_zero_copy/{$2="= 1"}1' > /etc/ecal/ecal.tmp | \
    awk -F"=" '/^memfile_buffer_count/{$2="= 5"}1' > /etc/ecal/ecal.tmp && \
	rm /etc/ecal/ecal.ini && \
	mv /etc/ecal/ecal.tmp /etc/ecal/ecal.ini

# Print the eCAL config
RUN ecal_config

ARG src
ARG BUILDPLATFORM

# Install libtorch and pytorch
COPY requirements*.txt /
RUN if [ "$src" = "cpu-torch" ]; then \
        python3 -m pip install --index-url https://download.pytorch.org/whl/cpu -r requirements_torch.txt \
        && python3 -m pip install --index-url https://pypi.python.org/simple -r requirements_yolo.txt \
        && if [ "$BUILDPLATFORM" = "linux/amd642" ]; then \
              wget -c https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcpu.zip --output-document libtorch.zip \
              && unzip libtorch.zip -d ~/src; \
           else \
              git clone --recurse-submodules -j8 https://github.com/pytorch/pytorch.git \
              && python3 -m pip install -r pytorch/requirements.txt \
              && python3 -m pip install mkl-static mkl-include \
              && export _GLIBCXX_USE_CXX11_ABI=1 \
              && export USE_CUDA=0 && export USE_ROCM=0 \
              && export BUILD_TEST=0 && export USE_KINETO=0 \
              && export USE_NUMPY=0 && export USE_ITT=0 \
              && export USE_MKLDNN=1 && export USE_MKLDNN_ACL=1 \
              && export MKLDNN_CPU_RUNTIME=TBB && USE_STATIC_MKL=1 \
              && export USE_DISTRIBUTED=0 && export USE_OPENMP=1 \
              && export USE_MKLDNN=1 && export MKLDNN_CPU_RUNTIME=TBB \
              && export BLAS=MKL && export MKL_THREADING=TBB \
              && export USE_TBB=1 && export USE_SYSTEM_TBB=1 \
              && export CMAKE_PREFIX_PATH=/usr/local/lib/python3.10/dist-packages \
              && python3 -u pytorch/tools/build_libtorch.py --rerun-cmake \
              && mkdir -p ~/src/libtorch \
              && mv /pytorch/torch/bin ~/src/libtorch/ \
              && mv /pytorch/torch/include ~/src/libtorch/ \
              && mv /pytorch/torch/lib ~/src/libtorch/ \
              && mv /pytorch/torch/share ~/src/libtorch/ \
              && rm -r /build \
              && rm -r /pytorch; \
           fi; \
    elif [ "$src" = "cuda-torch" ]; then \
        python3 -m pip install --index-url https://download.pytorch.org/whl/cu121 -r requirements_torch.txt \
        && python3 -m pip install --index-url https://pypi.python.org/simple -r requirements_yolo.txt \
        && wget -c https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcu121.zip --output-document libtorch.zip \
        && unzip libtorch.zip -d ~/src; \
    elif [ "$src" = "rocm-torch" ]; then \
        python3 -m pip install --index-url https://download.pytorch.org/whl/rocm6.0 -r requirements_torch.txt \
        && python3 -m pip install --index-url https://pypi.python.org/simple -r requirements_yolo.txt \
        && wget -c https://download.pytorch.org/libtorch/rocm6.0/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Brocm6.0.zip --output-document libtorch.zip \
        && unzip libtorch.zip -d ~/src; \
    fi

ENV PYTHONPATH "${PYTHONPATH}:/src"
WORKDIR /src
