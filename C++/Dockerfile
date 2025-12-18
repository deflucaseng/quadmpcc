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
    python3-venv \
    python3-numpy \
    libgtest-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup Python virtual environment
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Install Python packages in the virtual environment
RUN pip install --no-cache-dir \
    pre-commit \
    cpplint \
    black \
    flake8 \
    numpy


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
