import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterValue
from rclpy.parameter import Parameter, ParameterType


class JointAnglesParamNode(Node):
    def __init__(self):
        super().__init__('joint_angles_param')

        # Declare the 'joint_angles' parameter with an initial value
        self.declare_parameter('joint_angles', [1.0] * 12)
        self.get_logger().info(f"Initial joint angles: {self.get_parameter('joint_angles').value}")

        # Create services for getting and setting the parameter
        self.get_parameters_service = self.create_service(
            GetParameters, '/joint_angles_param/get_parameters', self.get_parameters_callback)

        self.set_parameters_service = self.create_service(
            SetParameters, '/joint_angles_param/set_parameters', self.set_parameters_callback)

        self.get_logger().info('JointAnglesParamNode is ready.')

    def get_parameters_callback(self, request, response):
        self.get_logger().info('Get Parameters service called.')

        # Fetch the joint_angles parameter value
        try:
            param_value = self.get_parameter('joint_angles').value
            param_value_msg = ParameterValue()
            param_value_msg.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            param_value_msg.double_array_value = param_value
            response.values.append(param_value_msg)
            self.get_logger().info(f"Providing joint angles: {param_value}")
        except Exception as e:
            self.get_logger().error(f"Error fetching joint_angles: {str(e)}")

        return response

    def set_parameters_callback(self, request, response):
        self.get_logger().info('Set Parameters service called.')

        # Iterate through the parameters in the request
        for param in request.parameters:
            if param.name == 'joint_angles' and param.value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
                try:
                    # Update the joint_angles parameter
                    self.set_parameters([Parameter(name='joint_angles', value=param.value.double_array_value)])
                    self.get_logger().info(f"Joint angles updated to: {param.value.double_array_value}")
                except Exception as e:
                    self.get_logger().error(f"Failed to set joint_angles: {str(e)}")
            else:
                self.get_logger().warning(f"Ignoring parameter: {param.name}")

        return response


def main(args=None):
    rclpy.init(args=args)
    joint_angles_param_node = JointAnglesParamNode()

    # Spin the node to keep it active
    rclpy.spin(joint_angles_param_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
