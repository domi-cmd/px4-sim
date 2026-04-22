# PX4 Autopilot with TigerVNC and NoVNC
FROM domicmd/ros2-jazzy-gazebo-harmonic:latest

# Metadata
LABEL description="ROS 2 Jazzy with Gazebo Harmonic, PX4 Autopilot, MAVROS, and NoVNC"
LABEL version="1.0"

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# ============================================================================
# Install PX4 Autopilot dependencies
# ============================================================================
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-jinja2 \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-packaging \
    astyle \
    exiftool \
    genromfs \
    kconfig-frontends \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libeigen3-dev \
    libopencv-dev \
    libxml2-utils \
    pkg-config \
    protobuf-compiler \
    geographiclib-tools \
    rsync \
    unzip \
    zip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for PX4
RUN pip3 install --no-cache-dir --break-system-packages \
    kconfiglib \
    jsonschema \
    pyros-genmsg \
    lxml

# ============================================================================
# Clone and build PX4 Autopilot
# ============================================================================
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive \
    && cd PX4-Autopilot 

# WORKDIR /root/PX4-Autopilot
# RUN DONT_RUN=1 make px4_sitl gz_x500

# Install Mavros package
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros

# Install dependencies for Mavros
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# ============================================================================
# Install TigerVNC and NoVNC
# ============================================================================
RUN apt-get update && apt-get install -y \
    xfce4 \
    xfce4-goodies \
    dbus-x11 \
    supervisor \
    tigervnc-standalone-server \
    tigervnc-common \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# Setup VNC
RUN mkdir -p /root/.vnc \
    && echo "1234" | vncpasswd -f > /root/.vnc/passwd \
    && chmod 600 /root/.vnc/passwd

# Create VNC xstartup script
RUN echo '#!/bin/sh\n\
unset SESSION_MANAGER\n\
unset DBUS_SESSION_BUS_ADDRESS\n\
exec startxfce4' > /root/.vnc/xstartup \
    && chmod +x /root/.vnc/xstartup

# ============================================================================
# Setup environment and directories
# ============================================================================
ENV DISPLAY=:1
RUN mkdir -p /var/log/supervisor

# Create supervisor configuration
RUN echo '[supervisord]\n\
nodaemon=true\n\
user=root\n\
\n\
[program:vncserver]\n\
command=/usr/bin/vncserver :1 -geometry 1920x1080 -depth 24 -localhost no -fg\n\
autorestart=true\n\
priority=100\n\
stdout_logfile=/var/log/supervisor/vncserver.log\n\
stderr_logfile=/var/log/supervisor/vncserver_error.log\n\
\n\
[program:novnc]\n\
command=/usr/share/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 6080\n\
autorestart=true\n\
priority=200\n\
stdout_logfile=/var/log/supervisor/novnc.log\n\
stderr_logfile=/var/log/supervisor/novnc_error.log' > /etc/supervisor/conf.d/supervisord.conf

# ============================================================================
# Setup environment variables
# ============================================================================
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo 'export GZ_SIM_RESOURCE_PATH=/root/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH}' >> /root/.bashrc \
    && echo 'export PX4_DIR=/root/PX4-Autopilot' >> /root/.bashrc

# ============================================================================
# Expose ports
# ============================================================================
# EXPOSE 5901
# EXPOSE 6080 
# EXPOSE 14540/udp
# EXPOSE 14550/udp
EXPOSE 14556/udp
EXPOSE 14557/udp
# EXPOSE 18570/udp

# Setup entrypoint
COPY ./px4_entrypoint.sh /
ENTRYPOINT ["/px4_entrypoint.sh"]


# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
    CMD curl -f http://localhost:6080/ || exit 1