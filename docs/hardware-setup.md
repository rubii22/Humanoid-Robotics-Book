# Hardware Setup Guide - Humanoid Robot Development Kit

## Overview

This guide provides detailed instructions for setting up the physical hardware components for humanoid robot development. The guide covers the recommended "Edge Kit" configuration including Jetson Orin Nano/NX, Intel RealSense cameras, and USB microphone arrays. This setup is designed for developing, testing, and deploying humanoid robot applications with real-world perception and interaction capabilities.

## Hardware Requirements

### Minimum Configuration
- **Compute Platform**: Jetson Orin Nano (8GB) or Jetson Orin NX (8GB)
- **Camera System**: Intel RealSense D435i or D455
- **Audio System**: USB microphone array (ReSpeaker 4-Mic Array or similar)
- **Power System**: 12V/10A power supply with voltage regulators
- **Connectivity**: Wi-Fi 6 or Ethernet connection

### Recommended Configuration
- **Compute Platform**: Jetson Orin NX (16GB) or Jetson Orin AGX (32GB)
- **Camera System**: Intel RealSense D455 (preferred for better depth accuracy)
- **Audio System**: ReSpeaker 6-Mic Circular Array v2.0
- **Additional Sensors**: IMU (BNO055), LiDAR (if needed), force/torque sensors
- **Power System**: Smart battery management with monitoring
- **Connectivity**: Wi-Fi 6 + Ethernet + optional 5G module

## Jetson Orin Setup

### Initial Setup

1. **Prepare the Jetson Board**
   - Ensure you have the Jetson carrier board with appropriate cooling
   - Connect power supply (official 19V/6.32A adapter recommended)
   - Connect HDMI display and USB keyboard for initial setup

2. **Install JetPack SDK**
   ```bash
   # Download JetPack SDK from NVIDIA Developer website
   # Follow the installation wizard
   # Select components:
   # - L4T BSP
   # - DeepStream SDK
   # - Isaac ROS packages
   # - CUDA Toolkit
   # - cuDNN
   # - TensorRT
   ```

3. **Configure System**
   ```bash
   # Update system
   sudo apt update && sudo apt upgrade -y

   # Install essential packages
   sudo apt install -y build-essential cmake git vim htop iotop
   sudo apt install -y python3-pip python3-dev python3-venv
   sudo apt install -y ros-humble-desktop
   ```

4. **Set up ROS 2 Environment**
   ```bash
   # Source ROS 2
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc

   # Initialize rosdep
   sudo rosdep init
   rosdep update
   ```

### Performance Configuration

```bash
# Set Jetson to maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Configure swap space for memory-intensive operations
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

## Intel RealSense Setup

### Installation

```bash
# Add RealSense repository
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380B84C6C5E2D6D640B
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install RealSense packages
sudo apt update
sudo apt install librealsense2-dkms
sudo apt install librealsense2-dev librealsense2-utils
sudo apt install ros-humble-realsense2-camera
```

### Configuration for Humanoid Robotics

```bash
# Create RealSense configuration file
sudo mkdir -p /etc/udev/rules.d/
sudo cp /usr/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Testing RealSense

```bash
# Test camera functionality
realsense-viewer

# Test ROS 2 integration
ros2 launch realsense2_camera rs_launch.py
```

## Audio System Setup

### Microphone Array Installation

```bash
# Install audio dependencies
sudo apt install -y pulseaudio alsa-utils
sudo apt install -y python3-pyaudio python3-speechrecognition

# For ReSpeaker microphone arrays specifically
git clone https://github.com/respeaker/usb_4_mic_array.git
cd usb_4_mic_array
sudo python3 setup.py install

# Install additional audio processing libraries
pip3 install webrtcvad
pip3 install pyaudio
pip3 install speechrecognition
```

### Audio Configuration

```bash
# Create PulseAudio configuration for microphone array
cat << EOF | sudo tee /etc/pulse/default.pa.d/respeaker.conf
### Custom configuration for ReSpeaker microphone array
load-module module-alsa-source device=hw:1,0 rate=16000 channels=4
set-default-source alsa_input.hw_1_0
EOF

# Restart PulseAudio
pulseaudio -k
pulseaudio --start
```

### Testing Audio System

```bash
# List audio devices
arecord -l

# Test recording
arecord -D hw:1,0 -f S16_LE -r 16000 -c 4 -d 5 test.wav

# Play back to verify
aplay test.wav
```

## Power System Setup

### Power Distribution Board

For humanoid robots, a clean power distribution system is crucial:

```bash
# Create a script to monitor power consumption
cat << 'EOF' > ~/power_monitor.py
#!/usr/bin/env python3

import time
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')

        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.power_pub = self.create_publisher(Float32, '/power_consumption', 10)

        self.timer = self.create_timer(1.0, self.monitor_power)

    def monitor_power(self):
        # Get system power consumption (simplified)
        try:
            # This would interface with actual power monitoring hardware
            # For now, simulate based on system load
            result = subprocess.run(['vcgencmd', 'measure_temp'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                temp_str = result.stdout.split('=')[1].split("'")[0]
                temperature = float(temp_str)
            else:
                temperature = 0.0

            # Publish battery state (simulated)
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()
            battery_msg.voltage = 12.6  # Simulated voltage
            battery_msg.temperature = temperature
            battery_msg.percentage = 85.0  # Simulated percentage
            battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

            self.battery_pub.publish(battery_msg)

        except Exception as e:
            self.get_logger().error(f'Error monitoring power: {e}')

def main(args=None):
    rclpy.init(args=args)
    monitor = PowerMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x ~/power_monitor.py
```

## Sensor Integration

### IMU Setup (BNO055)

```bash
# Install I2C tools
sudo apt install -y i2c-tools

# Enable I2C interface
sudo usermod -aG i2c $USER

# Test IMU connection
sudo i2cdetect -y -r 1

# Install BNO055 ROS 2 driver (if available) or use generic IMU driver
sudo apt install ros-humble-imu-tools
```

### Creating Sensor Configuration File

```yaml
# File: ~/robot_ws/src/hardware_interfaces/config/sensors.yaml
# Sensor configuration for humanoid robot

sensors:
  realsense_camera:
    type: "intel_realsense_d455"
    frame_id: "camera_link"
    parameters:
      enable_color: true
      enable_depth: true
      color_width: 1280
      color_height: 720
      depth_width: 1280
      depth_height: 720
      fps: 30

  imu:
    type: "bno055"
    frame_id: "imu_link"
    parameters:
      update_rate: 100
      linear_acceleration_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
      angular_velocity_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
      orientation_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

  microphones:
    type: "usb_microphone_array"
    frame_id: "microphone_array_link"
    parameters:
      channels: 4
      sample_rate: 16000
      bits_per_sample: 16
```

## Network and Connectivity Setup

### Wi-Fi Configuration

```bash
# Create network configuration for reliable connectivity
cat << EOF | sudo tee /etc/netplan/99-wifi-config.yaml
network:
  version: 2
  wifis:
    wlan0:
      access-points:
        "YOUR_NETWORK_NAME":
          password: "YOUR_PASSWORD"
      dhcp4: true
      optional: true
EOF

# Apply configuration
sudo netplan apply
```

### Ethernet Configuration (Recommended for Stability)

```bash
# Static IP configuration for robot-to-PC communication
cat << EOF | sudo tee /etc/netplan/99-ethernet-config.yaml
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
EOF

sudo netplan apply
```

## ROS 2 Hardware Interface

### Creating Hardware Abstraction Layer

```python
# File: ~/robot_ws/src/hardware_interfaces/hardware_interfaces/hardware_interface.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import threading
import time


class HardwareInterface(Node):
    def __init__(self):
        super().__init__('hardware_interface')

        # Publishers for sensor data
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.status_pub = self.create_publisher(String, '/hardware_status', 10)

        # Initialize hardware components
        self.initialize_hardware()

        # Timer for sensor data publishing
        self.sensor_timer = self.create_timer(0.02, self.publish_sensor_data)  # 50Hz

        self.get_logger().info('Hardware Interface initialized')

    def initialize_hardware(self):
        """Initialize all hardware components"""
        self.get_logger().info('Initializing hardware components...')

        # Initialize RealSense camera
        self.initialize_camera()

        # Initialize IMU
        self.initialize_imu()

        # Initialize audio system
        self.initialize_audio()

        # Initialize power monitoring
        self.initialize_power_monitoring()

        self.hardware_initialized = True
        self.get_logger().info('All hardware components initialized')

    def initialize_camera(self):
        """Initialize Intel RealSense camera"""
        # In practice, initialize camera drivers here
        self.get_logger().info('RealSense camera initialized')

    def initialize_imu(self):
        """Initialize IMU sensor"""
        # In practice, initialize IMU drivers here
        self.get_logger().info('IMU initialized')

    def initialize_audio(self):
        """Initialize microphone array"""
        # In practice, initialize audio drivers here
        self.get_logger().info('Audio system initialized')

    def initialize_power_monitoring(self):
        """Initialize power monitoring"""
        # In practice, initialize power monitoring hardware
        self.get_logger().info('Power monitoring initialized')

    def publish_sensor_data(self):
        """Publish sensor data from all hardware components"""
        if not self.hardware_initialized:
            return

        # Publish joint states (simulated for now)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3']
        joint_msg.position = [0.0, 0.0, 0.0]
        joint_msg.velocity = [0.0, 0.0, 0.0]
        joint_msg.effort = [0.0, 0.0, 0.0]

        self.joint_state_pub.publish(joint_msg)

        # Publish IMU data (simulated for now)
        imu_msg = Imu()
        imu_msg.header.stamp = joint_msg.header.stamp
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.w = 1.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.z = 9.81

        self.imu_pub.publish(imu_msg)

    def get_hardware_status(self):
        """Get overall hardware status"""
        status = {
            'timestamp': self.get_clock().now().nanoseconds,
            'camera': 'connected',
            'imu': 'connected',
            'audio': 'connected',
            'power': 'normal',
            'temperature': 'normal'
        }
        return status


def main(args=None):
    rclpy.init(args=args)
    hardware_interface = HardwareInterface()

    try:
        rclpy.spin(hardware_interface)
    except KeyboardInterrupt:
        hardware_interface.get_logger().info('Shutting down hardware interface')
    finally:
        hardware_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Hardware Interface Package Configuration

```xml
<!-- File: ~/robot_ws/src/hardware_interfaces/package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>hardware_interfaces</name>
  <version>0.0.1</version>
  <description>Hardware interfaces for humanoid robot</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

```python
# File: ~/robot_ws/src/hardware_interfaces/setup.py
from setuptools import find_packages, setup

package_name = 'hardware_interfaces'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Hardware interfaces for humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_interface = hardware_interfaces.hardware_interface:main',
        ],
    },
)
```

## Safety and Monitoring Systems

### Emergency Stop Implementation

```python
# File: ~/robot_ws/src/hardware_interfaces/hardware_interfaces/emergency_stop.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO  # Use appropriate GPIO library for Jetson


class EmergencyStop(Node):
    def __init__(self):
        super().__init__('emergency_stop')

        # Publishers
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.cmd_vel_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/safety_status', 10)

        # Initialize GPIO for hardware emergency stop button
        self.setup_gpio()

        # Timer for safety checks
        self.safety_timer = self.create_timer(0.1, self.check_safety)

        self.emergency_stop_active = False
        self.last_stop_time = self.get_clock().now()

        self.get_logger().info('Emergency Stop system initialized')

    def setup_gpio(self):
        """Setup GPIO for emergency stop button"""
        # Note: Jetson GPIO setup is different from Raspberry Pi
        # This is a conceptual example - actual implementation depends on carrier board
        try:
            # Initialize GPIO (this is conceptual - actual Jetson GPIO setup varies)
            # GPIO.setmode(GPIO.BOARD)  # or GPIO.BCM
            # GPIO.setup(EMERGENCY_STOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.get_logger().info('GPIO for emergency stop configured')
        except Exception as e:
            self.get_logger().error(f'Error setting up GPIO: {e}')

    def check_safety(self):
        """Check safety conditions"""
        # Check for emergency stop condition
        # This would read from actual hardware button or safety system
        stop_requested = self.check_emergency_stop()

        if stop_requested and not self.emergency_stop_active:
            self.activate_emergency_stop()
        elif not stop_requested and self.emergency_stop_active:
            self.deactivate_emergency_stop()

        # Publish safety status
        status_msg = String()
        status_msg.data = f"EMERGENCY_STOP:{self.emergency_stop_active}"
        self.status_pub.publish(status_msg)

    def check_emergency_stop(self):
        """Check if emergency stop is activated"""
        # In practice, read from hardware button
        # return not GPIO.input(EMERGENCY_STOP_PIN)  # Active low button
        return False  # Simulated - no emergency stop in simulation

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_stop_active = True
        self.last_stop_time = self.get_clock().now()

        # Publish emergency stop signal
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # Stop all movement
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_stop_pub.publish(cmd_msg)

        self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop"""
        self.emergency_stop_active = False
        self.get_logger().info('Emergency stop deactivated')


def main(args=None):
    rclpy.init(args=args)
    emergency_stop = EmergencyStop()

    try:
        rclpy.spin(emergency_stop)
    except KeyboardInterrupt:
        emergency_stop.get_logger().info('Shutting down emergency stop system')
    finally:
        emergency_stop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Calibration Procedures

### Camera Calibration

```bash
# Calibrate RealSense camera
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.0245 \
    --approximate 0.1 \
    image:=/camera/color/image_raw \
    camera:=/camera

# Or use the calibration tool provided by Intel
realsense-viewer
# Use the built-in calibration tools in the viewer
```

### IMU Calibration

```bash
# For BNO055, calibration typically involves:
# 1. Gathering data in different orientations
# 2. Using the sensor's internal calibration features
# 3. Saving calibration data to persistent storage

# Example ROS 2 node for IMU calibration would go here
```

## System Integration Testing

### Hardware Test Script

```python
# File: ~/robot_ws/src/hardware_interfaces/test/hardware_test.py

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class HardwareTest(Node):
    def __init__(self):
        super().__init__('hardware_test')

        # Create subscribers to verify sensor data
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.camera_callback, 10)

        # Create publisher for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sensor_data_received = {
            'joint_states': False,
            'imu': False,
            'camera': False
        }

        self.get_logger().info('Hardware test node initialized')

    def joint_callback(self, msg):
        """Verify joint state data"""
        if len(msg.position) > 0:
            self.sensor_data_received['joint_states'] = True
            self.get_logger().info('Joint states received successfully')

    def imu_callback(self, msg):
        """Verify IMU data"""
        if msg.orientation.w != 0:
            self.sensor_data_received['imu'] = True
            self.get_logger().info('IMU data received successfully')

    def camera_callback(self, msg):
        """Verify camera data"""
        if msg.height > 0 and msg.width > 0:
            self.sensor_data_received['camera'] = True
            self.get_logger().info('Camera data received successfully')

    def run_tests(self):
        """Run comprehensive hardware tests"""
        self.get_logger().info('Starting hardware integration tests...')

        # Wait for sensor data
        timeout = time.time() + 60*2  # 2 minutes timeout
        while not all(self.sensor_data_received.values()):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() > timeout:
                self.get_logger().error('Timeout waiting for sensor data')
                break

        # Test movement commands
        self.test_movement()

        # Print results
        self.print_test_results()

    def test_movement(self):
        """Test movement capabilities"""
        self.get_logger().info('Testing movement commands...')

        # Send forward command
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.2  # Move forward slowly
        self.cmd_vel_pub.publish(cmd_msg)
        time.sleep(2)

        # Stop
        cmd_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

        self.get_logger().info('Movement test completed')

    def print_test_results(self):
        """Print test results"""
        self.get_logger().info('=== HARDWARE TEST RESULTS ===')
        for sensor, received in self.sensor_data_received.items():
            status = 'PASS' if received else 'FAIL'
            self.get_logger().info(f'{sensor.upper()}: {status}')
        self.get_logger().info('=============================')


def main(args=None):
    rclpy.init(args=args)
    test_node = HardwareTest()

    try:
        test_node.run_tests()
    except KeyboardInterrupt:
        test_node.get_logger().info('Hardware test interrupted')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Troubleshooting Guide

### Common Hardware Issues

1. **RealSense Camera Not Detected**
   - Check USB 3.0 connection
   - Verify udev rules are properly installed
   - Try different USB port
   - Check power supply adequacy

2. **Audio System Not Working**
   - Verify microphone array is properly connected
   - Check audio permissions: `sudo usermod -a -G audio $USER`
   - Test with: `arecord -l` to list audio devices
   - Check PulseAudio configuration

3. **IMU Communication Issues**
   - Verify I2C is enabled: `sudo usermod -aG i2c $USER`
   - Check I2C address: `sudo i2cdetect -y -r 1`
   - Verify wiring connections

4. **Power Management Problems**
   - Monitor system temperature: `sudo tegrastats`
   - Check for thermal throttling
   - Verify power supply capacity
   - Monitor current consumption

### Performance Optimization

```bash
# Optimize system for real-time performance
sudo jetson_clocks --show  # Check current clock settings

# Set performance mode
sudo nvpmodel -m 0  # Maximum performance

# Monitor system performance
sudo tegrastats &    # Real-time stats
htop                # CPU/Memory usage
iotop              # I/O monitoring
```

## Maintenance and Updates

### System Updates

```bash
# Regular system maintenance
sudo apt update && sudo apt upgrade -y

# Update RealSense firmware if needed
sudo apt install librealsense2-dev librealsense2-utils
# Use realsense-update-tool for firmware updates

# Backup system configuration
tar -czf ~/robot_backup_$(date +%Y%m%d).tar.gz \
    ~/.bashrc \
    /etc/netplan/ \
    ~/robot_ws/src/hardware_interfaces/
```

## Summary

This hardware setup guide provides comprehensive instructions for configuring the physical components of a humanoid robot development system. The setup includes:

1. **Jetson Orin Platform**: High-performance computing for AI and robotics
2. **Intel RealSense**: Advanced depth and RGB sensing
3. **Audio System**: Multi-microphone array for voice interaction
4. **Power Management**: Reliable power distribution and monitoring
5. **Safety Systems**: Emergency stop and monitoring capabilities
6. **Connectivity**: Robust network configuration

The configuration is designed to support the complete Physical AI & Humanoid Robotics curriculum, enabling students to develop, test, and deploy sophisticated humanoid robot applications with real-world perception and interaction capabilities.

Following this guide will result in a fully functional humanoid robot development platform capable of executing the projects and exercises outlined in the course modules.