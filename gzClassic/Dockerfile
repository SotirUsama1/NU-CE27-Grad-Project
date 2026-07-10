FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install prerequisites
RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

# Fetch the OSRF repository key
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

# Add the Gazebo stable repository for Ubuntu Focal (20.04)
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable focal main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Classic 11
RUN apt-get update && apt-get install -y \
    gazebo11 \
    && rm -rf /var/lib/apt/lists/*

# Default command to run Gazebo Classic
CMD ["gazebo"]
