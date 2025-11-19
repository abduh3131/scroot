#!/usr/bin/env python3
import rospy
import subprocess
import glob
import time

def find_serial_port():
    """Finds the first available /dev/ttyUSB* port."""
    ports = glob.glob('/dev/ttyUSB*')
    if ports:
        return ports[0]
    return None

if __name__ == '__main__':
    rospy.init_node('rplidar_starter', anonymous=True)
    
    port = find_serial_port()
    
    if port:
        rospy.loginfo(f"Found RPLiDAR on port {port}. Starting rplidarNode...")
        # Construct the rosrun command
        command = [
            'rosrun', 'rplidar_ros', 'rplidarNode',
            f'_serial_port:={port}',
            '_serial_baudrate:=115200',
            '_frame_id:=laser_frame',
            '_inverted:=false',
            '_angle_compensate:=true'
        ]
        
        rospy.loginfo("Waiting 2 seconds for RPLiDAR to power on...")
        time.sleep(2)
        
        try:
            # Use subprocess.Popen to run the node
            proc = subprocess.Popen(command)
            # Wait for the node to finish
            proc.wait()
        except rospy.ROSInterruptException:
            proc.terminate()
        except Exception as e:
            rospy.logerr(f"Failed to start rplidarNode: {e}")
    else:
        rospy.logwarn("No RPLiDAR detected on /dev/ttyUSB*. Skipping startup.")
        # Keep the node alive for a moment to ensure the warning is logged
        time.sleep(1)
