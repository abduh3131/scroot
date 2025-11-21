# File and Node Map

```
catkin_ws/
├─ src/sensor_interface/
│  └─ scripts/sensor_interface_node.py        # Publishes fused SensorHub messages with camera + LiDAR
│  └─ scripts/visualize_sensor_hub.py         # Optional debug viewer for raw SensorHub output
├─ src/scooter_control/
│  ├─ msg/
│  │  ├─ Detection*.msg                       # Object detection messages
│  │  └─ ActuatorCommand.msg                  # Throttle / brake / steering command stream
│  ├─ scripts/
│  │  ├─ dual_yolo_node.py                    # Standalone YOLO annotator for camera frames
│  │  ├─ sensor_fusion_planner_node.py        # Subscribes to /sensor_hub/data, fuses LiDAR + YOLO, publishes drive plan
│  │  ├─ controller_node.py                   # Consumes planner commands and streams actuator values to file + topic
│  │  └─ live_overlay_gui.py                  # Lightweight GUI for overlay stream and actuator HUD
│  └─ launch/                                 # Add launch files here to run nodes together
└─ src/scroot_bridge/                         # Wrapper for external scroot bridge (unchanged)
```

## Topic Flow
```
/sensor_hub/data (SensorHub)
        │
        ▼
planner: sensor_fusion_planner_node
    ├─ /planner/detections (DetectionArray)
    ├─ /planner/overlay (Image)
    └─ /controller/command (ActuatorCommand)
        │
        ▼
controller: controller_node
    └─ /controller/actuator_values (ActuatorCommand) + ~/controller_command.txt
        │
        ▼
GUI: live_overlay_gui subscribes to /planner/overlay + /controller/actuator_values
```
