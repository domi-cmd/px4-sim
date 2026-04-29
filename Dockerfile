# Use the image you found via inspect
FROM erdemuysalx/px4-sitl:latest

# Create the folder for the planner
RUN mkdir -p /root/planner_ws

# Copy the script from the Windows folder into the image
COPY planner_ws/local_planner_node.py /root/planner_ws/local_planner_node.py

# Make the script executable just in case
RUN chmod +x /root/planner_ws/local_planner_node.py

# Set the working directory to the workspace when log in
WORKDIR /root/planner_ws

# Standard practice for ROS2: Source the setup script automatically in bash
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc