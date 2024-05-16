# src: cpu, cuda, rocm
ARG src=cpu

FROM ubuntu:jammy AS cpu-base
ARG libtorch_link="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcpu.zip"

FROM nvidia/cuda:12.1.1-cudnn8-devel-ubuntu22.04 AS cuda-base
ARG libtorch_link="https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Bcu121.zip"

FROM rocm/dev-ubuntu-22.04:6.0-complete AS rocm-base
ARG libtorch_link="https://download.pytorch.org/libtorch/rocm6.0/libtorch-cxx11-abi-shared-with-deps-2.3.0%2Brocm6.0.zip"

FROM ${src}-base AS image
ARG src
ARG libtorch_link

# Install eCAL from PPA:
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
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -fr /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# flatbuffers-compiler-dev libflatbuffers-dev python3-flatbuffers

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

COPY requirements_$src.txt requirements_$src.txt
RUN python3 -m pip install -r requirements_$src.txt

COPY requirements.txt requirements.txt
RUN python3 -m pip install -r requirements.txt

RUN wget -c $libtorch_link --output-document libtorch.zip \
    && unzip libtorch.zip -d ~/src

ENV PYTHONPATH "${PYTHONPATH}:/src"
WORKDIR /src
