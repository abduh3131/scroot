"""
kuksa_bridge_node.py
-------------------------------------
ROS 2 node that bridges ROS 2 vehicle topics to the Kuksa Databroker.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import asyncio
import threading
from kuksa_client.grpc.aio import VSSClient
from kuksa_client.grpc import Datapoint


class KuksaBridgeNode(Node):
    def __init__(self):
        super().__init__('kuksa_bridge_node')

        # Kuksa connection config
        self.kuksa_host = '127.0.0.1'
        self.kuksa_port = 55555
        self.update_interval = 1.0

        # Initialize vehicle data cache
        self.vehicle_data = {
            "Vehicle.OBD.VehicleSpeed": 0.0,
            "Vehicle.OBD.EngineSpeed": 0.0,
            "Vehicle.OBD.ThrottlePosition": 0.0,
            "Vehicle.OBD.CoolantTemperature": 0.0
        }

        # ROS 2 topic subscriptions
        self.create_subscription(Float32, '/vehicle/speed', self.speed_callback, 10)
        self.create_subscription(Float32, '/vehicle/engine_speed', self.engine_callback, 10)
        self.create_subscription(Float32, '/vehicle/throttle', self.throttle_callback, 10)
        self.create_subscription(Float32, '/vehicle/coolant_temp', self.coolant_callback, 10)

        self.get_logger().info("Kuksa Bridge Node initialized. Starting async connection...")

        self.bridge_thread = threading.Thread(target=self.run_async_loop, daemon=True)
        self.bridge_thread.start()

    # Callback functions
    def speed_callback(self, msg):
        self.vehicle_data["Vehicle.OBD.VehicleSpeed"] = float(msg.data)

    def engine_callback(self, msg):
        self.vehicle_data["Vehicle.OBD.EngineSpeed"] = float(msg.data)

    def throttle_callback(self, msg):
        self.vehicle_data["Vehicle.OBD.ThrottlePosition"] = float(msg.data)

    def coolant_callback(self, msg):
        self.vehicle_data["Vehicle.OBD.CoolantTemperature"] = float(msg.data)

    # Async bridge loop
    def run_async_loop(self):
        asyncio.run(self.kuksa_task())

    async def kuksa_task(self):
        try:
            async with VSSClient(self.kuksa_host, self.kuksa_port) as client:
                self.get_logger().info(f"Connected to Kuksa Databroker @ {self.kuksa_host}:{self.kuksa_port}")
                while rclpy.ok():
                    await client.set_current_values({
                        key: Datapoint(value) for key, value in self.vehicle_data.items()
                    })
                    self.get_logger().info(f"Pushed to Kuksa: {self.vehicle_data}")
                    await asyncio.sleep(self.update_interval)
        except Exception as e:
            self.get_logger().error(f"Kuksa Bridge error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = KuksaBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Kuksa Bridge Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

