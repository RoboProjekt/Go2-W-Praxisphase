#!/usr/bin/env python3
"""
Simple DDS <-> ROS2 bridge for Unitree GO2W SDK topics (strict DDS mode).

This version assumes Cyclone DDS and the SDK are installed and importable.
It will raise on missing dependencies.

Clone Git "https://github.com/unitreerobotics/unitree_sdk2_python"
Move "ros2_cyclone_bridge.py" to "unitree_sdk2_python/example/go2w/bridge/"
final result: ".../unitree_sdk2_python/example/go2w/bridge/ros2_cyclone_bridge.py"
"""
import json
import sys
import time
import threading
import os

# If the package isn't installed in the environment, add the repo root to sys.path
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC


class DdsToRos2Bridge(Node):
    def __init__(self):
        super().__init__('dds_to_ros2_bridge')

        # ROS2 publishers
        self.ros_imu_pub = self.create_publisher(Imu, 'unitree/imu', 10)
        self.ros_joint_pub = self.create_publisher(JointState, 'unitree/joint_states', 10)
        self.ros_raw_pub = self.create_publisher(String, 'unitree/lowstate_raw', 10)

        # ROS2 subscriber for desired joint commands -> forward to rt/lowcmd
        self.ros_cmd_sub = self.create_subscription(String, 'unitree/lowcmd_in', self.on_ros_lowcmd_in, 10)

        # DDS publishers/subscribers
        ChannelFactoryInitialize(0)

        # subscribe to lowstate
        self.lowstate_sub = ChannelSubscriber('rt/lowstate', LowState_)
        self.lowstate_sub.Init(self.on_lowstate, 10)

        # prepare publisher for lowcmd
        self.lowcmd_pub = ChannelPublisher('rt/lowcmd', LowCmd_)
        self.lowcmd_pub.Init()

        # CRC helper
        self.crc = CRC()

        self.get_logger().info('DDS <-> ROS2 bridge initialized')

    def on_lowstate(self, msg: LowState_):
        try:
            imu = Imu()
            q = msg.imu_state.quaternion
            if q and len(q) >= 4:
                imu.orientation.x = float(q[0])
                imu.orientation.y = float(q[1])
                imu.orientation.z = float(q[2])
                imu.orientation.w = float(q[3])

            g = msg.imu_state.gyroscope
            a = msg.imu_state.accelerometer
            if g and len(g) >= 3:
                imu.angular_velocity.x = float(g[0])
                imu.angular_velocity.y = float(g[1])
                imu.angular_velocity.z = float(g[2])
            if a and len(a) >= 3:
                imu.linear_acceleration.x = float(a[0])
                imu.linear_acceleration.y = float(a[1])
                imu.linear_acceleration.z = float(a[2])

            self.ros_imu_pub.publish(imu)
            # Echo what was translated
            try:
                first_qs = [round(ms.q, 4) for ms in msg.motor_state[:12]]
            except Exception:
                first_qs = None
            tick = int(msg.tick) if hasattr(msg, 'tick') else None
            self.get_logger().info(f'Translated LowState tick={tick} to IMU and JointState; motor_q[0:12]={first_qs}')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish IMU: {e}')

        try:
            js = JointState()
            names = []
            positions = []
            velocities = []
            efforts = []

            for i in range(min(12, len(msg.motor_state))):
                names.append(f'joint_{i}')
                positions.append(float(msg.motor_state[i].q))
                velocities.append(float(msg.motor_state[i].dq))
                efforts.append(float(msg.motor_state[i].tau_est if hasattr(msg.motor_state[i], 'tau_est') else 0.0))

            js.name = names
            js.position = positions
            js.velocity = velocities
            js.effort = efforts
            self.ros_joint_pub.publish(js)
            # echo jointstate translation
            self.get_logger().info(f'Published JointState names={js.name} positions={js.position}')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish JointState: {e}')

        try:
            raw = {}
            raw['tick'] = int(msg.tick) if hasattr(msg, 'tick') else 0
            raw['power_v'] = float(msg.power_v) if hasattr(msg, 'power_v') else 0.0
            raw['motor_q'] = [float(ms.q) for ms in msg.motor_state[:12]]
            s = String()
            s.data = json.dumps(raw)
            self.ros_raw_pub.publish(s)
            self.get_logger().debug(f'Published raw LowState JSON: {s.data}')
        except Exception as e:
            self.get_logger().warn(f'Failed to publish raw JSON: {e}')

    def on_ros_lowcmd_in(self, msg: String):
        try:
            payload = json.loads(msg.data)
            targets = payload.get('q', [])
            kp = payload.get('kp', 70.0)
            kd = payload.get('kd', 5.0)

            lowcmd = unitree_go_msg_dds__LowCmd_()

            lowcmd.head[0] = 0xFE
            lowcmd.head[1] = 0xEF
            lowcmd.level_flag = 0xFF
            lowcmd.gpio = 0

            for i in range(20):
                lowcmd.motor_cmd[i].mode = 0x01
                if i < len(targets):
                    lowcmd.motor_cmd[i].q = float(targets[i])
                else:
                    lowcmd.motor_cmd[i].q = 0.0
                lowcmd.motor_cmd[i].dq = 0.0
                lowcmd.motor_cmd[i].kp = float(kp)
                lowcmd.motor_cmd[i].kd = float(kd)
                lowcmd.motor_cmd[i].tau = 0.0

            lowcmd.crc = self.crc.Crc(lowcmd)
            ok = self.lowcmd_pub.Write(lowcmd)
            # Echo what was sent
            try:
                q_snippet = [round(lowcmd.motor_cmd[i].q,4) for i in range(12)]
            except Exception:
                q_snippet = None
            self.get_logger().info(f'Received ROS lowcmd -> published LowCmd crc={lowcmd.crc} write_ok={ok} q[0:12]={q_snippet} kp={kp} kd={kd}')
        except Exception as e:
            self.get_logger().error(f'Invalid lowcmd JSON: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DdsToRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down bridge')
        node.lowstate_sub.Close()
        node.lowcmd_pub.Close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Simple DDS <-> ROS2 bridge for Unitree GO2W SDK topics.

This script listens on SDK DDS topics (uses Cyclone DDS via the SDK's channel layer)
and republishes simplified ROS2 messages. It also listens to a ROS2 topic for
High-level joint/command intents and forwards them to the SDK `rt/lowcmd` DDS topic.

Notes:
- Requires `rclpy` (ROS 2) installed in the same Python environment where you run this.
- Requires Cyclone DDS Python bindings and the SDK package (so `unitree_sdk2py` import works).
- This bridge provides a minimal mapping: LowState_ -> sensor_msgs/Imu + sensor_msgs/JointState + a raw JSON topic; and a simple ROS2 -> LowCmd publisher.
- Tweak topic names / mappings for your robot and simulator.
"""
import json
import sys
import time
import threading

# If the package isn't installed in the environment, add the repo root to sys.path
# so the local `unitree_sdk2py` package can be imported when running this script
# directly from the repository.
import os
if 'unitree_sdk2py' not in sys.modules:
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import String
except Exception as e:
    print("ROS2 (rclpy) import failed:", e)
    print("Make sure you run this script with a ROS2-enabled Python environment (rclpy installed).")
    raise

NO_DDS = False
try:
    # Import SDK DDS layer and IDL types
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, LowCmd_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_, unitree_go_msg_dds__LowState_
    from unitree_sdk2py.utils.crc import CRC
except Exception as e:
    # Cyclone DDS or SDK not available in this environment — fall back to ROS-only mode
    NO_DDS = True
    print('Warning: DDS imports failed (cyclonedds or SDK missing):', e)
    print('Bridge will run in ROS-only mode: DDS publish/subscribe disabled.')

    # Provide minimal dummy classes so rest of the code can run without changing logic
    class ChannelSubscriber:
        def __init__(self, *args, **kwargs):
            pass
        def Init(self, handler=None, queueLen: int = 0):
            # No-op: no DDS incoming messages
            print('ChannelSubscriber.Init called in NO_DDS mode — no DDS subscription created')
        def Close(self):
            pass

    class ChannelPublisher:
        def __init__(self, *args, **kwargs):
            pass
        def Init(self):
            print('ChannelPublisher.Init called in NO_DDS mode — publishes will be no-ops')
        def Write(self, sample, timeout: float = None):
            print('NO_DDS: ChannelPublisher.Write called — sample not sent to DDS')
            return True
        def Close(self):
            pass

    def ChannelFactoryInitialize(id: int = 0, networkInterface: str = None):
        print('NO_DDS: ChannelFactoryInitialize called — skipping DDS initialization')

    # Minimal placeholder types to satisfy type hints; these aren't used in NO_DDS mode
    class LowState_:
        pass
    class LowCmd_:
        pass
    def unitree_go_msg_dds__LowCmd_():
        # simple struct-like object with attributes used in code
        class _C:
            def __init__(self):
                self.head = [0, 0]
                self.level_flag = 0
                self.gpio = 0
                # motor_cmd array of simple objects
                class M: pass
                self.motor_cmd = [type('M', (), {'mode':0,'q':0.0,'dq':0.0,'kp':0.0,'kd':0.0,'tau':0.0,'reserve':[0,0,0]})() for _ in range(20)]
                self.crc = 0
        return _C()
    def unitree_go_msg_dds__LowState_():
        return LowState_()
    class CRC:
        def __init__(self):
            pass
        def Crc(self, msg):
            return 0


class DdsToRos2Bridge(Node):
    def __init__(self):
        super().__init__('dds_to_ros2_bridge')

        # ROS2 publishers
        self.ros_imu_pub = self.create_publisher(Imu, 'unitree/imu', 10)
        self.ros_joint_pub = self.create_publisher(JointState, 'unitree/joint_states', 10)
        self.ros_raw_pub = self.create_publisher(String, 'unitree/lowstate_raw', 10)

        # ROS2 subscriber for desired joint commands -> forward to rt/lowcmd
        self.ros_cmd_sub = self.create_subscription(String, 'unitree/lowcmd_in', self.on_ros_lowcmd_in, 10)

        # DDS publishers/subscribers
        # Make sure DDS is initialized before creating channels
        ChannelFactoryInitialize(0)

        # subscribe to lowstate
        self.lowstate_sub = ChannelSubscriber('rt/lowstate', LowState_)
        # use queueLen to let channel layer buffer samples
        self.lowstate_sub.Init(self.on_lowstate, 10)

        # prepare publisher for lowcmd (we will write messages when ROS sends commands)
        self.lowcmd_pub = ChannelPublisher('rt/lowcmd', LowCmd_)
        self.lowcmd_pub.Init()

        # CRC helper
        self.crc = CRC()

        self.get_logger().info('DDS <-> ROS2 bridge initialized')

    def on_lowstate(self, msg: LowState_):
        # Publish a simplified IMU message if present
        try:
            imu = Imu()
            # map quaternion (w,x,y,z) ordering: IDL gives quaternion as list [x,y,z,w]? Check your IDL.
            # Here we assume imu_state.quaternion = [x, y, z, w]
            q = msg.imu_state.quaternion
            if q and len(q) >= 4:
                imu.orientation.x = float(q[0])
                imu.orientation.y = float(q[1])
                imu.orientation.z = float(q[2])
                imu.orientation.w = float(q[3])

            g = msg.imu_state.gyroscope
            a = msg.imu_state.accelerometer
            if g and len(g) >= 3:
                imu.angular_velocity.x = float(g[0])
                imu.angular_velocity.y = float(g[1])
                imu.angular_velocity.z = float(g[2])
            if a and len(a) >= 3:
                imu.linear_acceleration.x = float(a[0])
                imu.linear_acceleration.y = float(a[1])
                imu.linear_acceleration.z = float(a[2])

            self.ros_imu_pub.publish(imu)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish IMU: {e}')

        # Publish joint states - map the first 12 motor_state entries to JointState message
        try:
            js = JointState()
            now_ns = self.get_clock().now().to_msg()
            # leave header stamp out (JointState has header optional)
            names = []
            positions = []
            velocities = []
            efforts = []

            for i in range(min(12, len(msg.motor_state))):
                names.append(f'joint_{i}')
                positions.append(float(msg.motor_state[i].q))
                velocities.append(float(msg.motor_state[i].dq))
                efforts.append(float(msg.motor_state[i].tau_est if hasattr(msg.motor_state[i], 'tau_est') else 0.0))

            js.name = names
            js.position = positions
            js.velocity = velocities
            js.effort = efforts
            self.ros_joint_pub.publish(js)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish JointState: {e}')

        # Publish raw JSON for debugging
        try:
            raw = {}
            raw['tick'] = int(msg.tick) if hasattr(msg, 'tick') else 0
            raw['power_v'] = float(msg.power_v) if hasattr(msg, 'power_v') else 0.0
            raw['motor_q'] = [float(ms.q) for ms in msg.motor_state[:12]]
            s = String()
            s.data = json.dumps(raw)
            self.ros_raw_pub.publish(s)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish raw JSON: {e}')

    def on_ros_lowcmd_in(self, msg: String):
        # Expect incoming message to be a JSON string with simple joint targets
        try:
            payload = json.loads(msg.data)
            targets = payload.get('q', [])
            kp = payload.get('kp', 70.0)
            kd = payload.get('kd', 5.0)

            # Basic validation
            if not isinstance(targets, list):
                raise ValueError("'q' must be a list of numbers")
            if len(targets) > 20:
                raise ValueError("'q' list length must be <= 20")
            # coerce to floats and validate elements
            try:
                targets = [float(x) for x in targets]
            except Exception:
                raise ValueError("All elements in 'q' must be numeric")

            lowcmd = unitree_go_msg_dds__LowCmd_()  # default initializer

            # set headers as SDK expects
            lowcmd.head[0] = 0xFE
            lowcmd.head[1] = 0xEF
            lowcmd.level_flag = 0xFF
            lowcmd.gpio = 0

            for i in range(20):
                lowcmd.motor_cmd[i].mode = 0x01
                if i < len(targets):
                    lowcmd.motor_cmd[i].q = float(targets[i])
                else:
                    lowcmd.motor_cmd[i].q = 0.0
                lowcmd.motor_cmd[i].dq = 0.0
                lowcmd.motor_cmd[i].kp = float(kp)
                lowcmd.motor_cmd[i].kd = float(kd)
                lowcmd.motor_cmd[i].tau = 0.0

            # compute CRC and publish
            try:
                lowcmd.crc = self.crc.Crc(lowcmd)
            except Exception:
                lowcmd.crc = 0

            ok = self.lowcmd_pub.Write(lowcmd)
            if not ok:
                self.get_logger().warn('Failed to write lowcmd to DDS')
        except Exception as e:
            # include a short snippet of the payload to help debugging
            snippet = (msg.data[:200] + '...') if isinstance(msg.data, str) and len(msg.data) > 200 else msg.data
            self.get_logger().error(f'Invalid lowcmd JSON: {e} -- payload snippet: {snippet}')


def main(args=None):
    rclpy.init(args=args)
    node = DdsToRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down bridge')
        node.lowstate_sub.Close()
        node.lowcmd_pub.Close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
