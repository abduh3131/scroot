# scroot/core

Core dataclasses and telemetry payloads that surface throughout the ROS
integration live here.  Importing `scroot.core` gives you the `LidarSnapshot`
metadata wrapper and `PilotTickData` structure so bridge code, GUI widgets, and
external dashboards agree on the schema for fused SensorHub information.
