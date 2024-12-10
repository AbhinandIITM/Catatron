import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

class JointAnglesParamNode(Node):
    def __init__(self):
        super().__init__('joint_angles_param')
        # Declare the 'joint_angles' parameter with an initial value
        self.declare_parameter('joint_angles', [0.0] * 12)
        self.get_logger().info(f"Initial joint angles: {self.get_parameter('joint_angles').value}")

    def set_joint_angles(self, new_angles):
        """
        Method to set new joint angles dynamically.
        """
        # Set the 'joint_angles' parameter with the new values
        self.set_parameters([Parameter('joint_angles', Parameter.Type.DOUBLE_ARRAY, new_angles)])
        self.get_logger().info(f"Joint angles updated to: {new_angles}")

    def get_joint_angles(self):
        """
        Method to get the current joint angles.
        """
        current_angles = self.get_parameter('joint_angles').value
        self.get_logger().info(f"Current joint angles: {current_angles}")
        return current_angles

def main(args=None):
    rclpy.init(args=args)
    joint_angles_param_node = JointAnglesParamNode()
    
    # Test updating joint angles (change values as needed)
    # joint_angles_param_node.set_joint_angles([0.1, 0.2, -0.3, 0.4, 0.5, -0.6, 0.7, -0.8, 0.9, -1.0, 1.1, -1.2])
    
    # Test getting the current joint angles
    # joint_angles_param_node.get_joint_angles()

    # Spin the node to keep it active
    rclpy.spin(joint_angles_param_node)

if __name__ == '__main__':
    main()
