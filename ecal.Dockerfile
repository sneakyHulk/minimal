# Base image:
FROM ubuntu:jammy

# Install eCAL from PPA:
RUN apt-get update && \
	apt-get install -y software-properties-common && \
	rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ecal/ecal-latest
RUN apt-get install -y ecal

# Install dependencies for compiling the hello world examples.
# You can omit this, if you don't want to build applications in the container.
RUN apt-get install -y cmake pkg-config g++ libprotobuf-dev protobuf-compiler \
    python3-msgpack libmsgpack-dev python3 python3-pip python3-ecal5 git gdb

# Set network_enabled = true in ecal.ini.
# You can omit this, if you only need local communication.
RUN awk -F"=" '/^network_enabled/{$2="= true"}1' /etc/ecal/ecal.ini | awk -F"=" '/^memfile_zero_copy/{$2="= 1"}1' > /etc/ecal/ecal.tmp && \
	rm /etc/ecal/ecal.ini && \
	mv /etc/ecal/ecal.tmp /etc/ecal/ecal.ini

# Print the eCAL config
RUN ecal_config