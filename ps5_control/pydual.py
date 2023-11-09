import rclpy
from rclpy.node import Node
from pydualsense import pydualsense, TriggerModes

class PS5ControllerNode(Node):

    def __init__(self):
        super().__init__('ps5_controller_node')
        self.ds = pydualsense()
        self.ds.init()

        if self.ds.device is None:
            self.get_logger().warn("No DualSense controller found!")
            rclpy.shutdown()

        self.ds.circle_pressed += self.circle_down
        self.ds.light.setColorI(255,0,0) # set touchpad color to red
        self.ds.triggerL.setMode(TriggerModes.Rigid)
        self.ds.triggerL.setForce(1, 255)

        # Set a timer to periodically check controller inputs
        # self.create_timer(0.01, self.check_controller_input)  # 100 Hz polling rate


    def circle_down(self, state):
        print(f'circle {state}')

    def close(self):
        self.ds.close()



def main(args=None):
    rclpy.init(args=args)
    node = PS5ControllerNode()
    rclpy.spin(node)
    node.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
