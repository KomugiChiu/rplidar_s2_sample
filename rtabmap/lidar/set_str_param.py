"""Set string-typed rtabmap params on a running node (bypasses ros2 CLI
YAML parsing which turns '0.4' into double and gets rejected)."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def main():
    node_name = sys.argv[1] if len(sys.argv) > 1 else '/rtabmap/icp_odometry'
    pairs = sys.argv[2:]
    rclpy.init()
    node = Node('param_setter')
    client = node.create_client(
        __import__('rcl_interfaces.srv', fromlist=['SetParameters']).SetParameters,
        node_name + '/set_parameters')
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('waiting for set_parameters service...')
    from rcl_interfaces.srv import SetParameters
    req = SetParameters.Request()
    while pairs:
        name = pairs.pop(0)
        value = pairs.pop(0)
        req.parameters.append(Parameter(name, value=value).to_parameter_msg())
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    for r in fut.result().results:
        print('ok' if r.successful else 'FAIL: ' + r.reason)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
