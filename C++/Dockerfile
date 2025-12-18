# Use the official Ubuntu 24.04 as the base image
FROM ubuntu:24.04

# Arguments for non-root user
ARG USERNAME=devuser
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Europe/Paris

# Install dependencies and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    sudo \
    python3 \
    python3-dev \
    python3-pip \
    python3-numpy \
    libgtest-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python packages needed (numpy is already apt-installed, but pinned version might be needed?)
# Keeping it simple for now, relying on apt's numpy or upgrading if necessary. 
# Removing matplotlib as per plan.
# RUN pip3 install numpy==1.20.0 

# Install CppAD
WORKDIR /tmp
RUN git clone https://github.com/coin-or/CppAD.git && \
    cd CppAD && \
    mkdir build && cd build && \
    cmake .. -Dcppad_prefix=/usr/local && \
    make install && \
    cd ../.. && rm -rf CppAD

# Install CppADCodeGen
RUN git clone https://github.com/joaoleal/CppADCodeGen.git && \
    cd CppADCodeGen && \
    mkdir build && cd build && \
    cmake .. && \
    make install && \
    cd ../.. && rm -rf CppADCodeGen

# Install blasfeo
RUN git clone https://github.com/giaf/blasfeo.git && \
    cd blasfeo && \
    mkdir build && cd build && \
    # cmake .. && \
    # make install && \
    cd ../..

# Install hpipm
RUN git clone https://github.com/giaf/hpipm.git && \
    cd hpipm && \
    # make static_library TARGET=ARMV8A_APPLE_M1 -j4 && \
    # mkdir -p /app/hpipm && \
    # cp -r include /app/hpipm/ && \
    # cp -r lib /app/hpipm/ && \
    cd ..

# Setup non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set working directory and ownership
WORKDIR /app/MPCC
RUN chown -R $USERNAME:$USERNAME /app

# Switch to non-root user
USER $USERNAME

# Define volume
VOLUME /app/MPCC

CMD ["/bin/bash"]
