import rclpy
import time
import board
import busio
import adafruit_bno055

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64

class IMUPublisher(Node):

    def __init__(self):
        super().__init__('node_encoder_publisher')
        
        self.imuPub = self.create_publisher(Float64, 'IMU', 10)
        timer_period = 0.010  # 10 milliseconds
        self.timer = self.create_timer(timer_period, self.timer_callback)               # IMU subscriber callback in every 10 milliseconds
        self.i2c = busio.I2C(board.SCL, board.SDA)					# Initialise Jetson NANO onboard IMU bus
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)				# Initialise the BNO055 IMU with I2C port
        self.yaw = Float64()
        self.get_logger().info('Node IMU Publisher started')
    
    def timer_callback(self):
        # convert IMU value range from 0-360 to [0 to 180, -180 to 0] as required for mapping algorithm
        yaw,pitch,roll = self.sensor.euler
        if yaw > 180:
            yaw = 360 - yaw
        elif yaw < 180:
            yaw = yaw * (-1)
  
        self.yaw.data = yaw
	
        self.imuPub.publish(self.yaw)                                                    # Publish the IMU value as ros topic 
    
def main(args=None):
    rclpy.init(args=args)
    IMU_publisher = IMUPublisher()

    rclpy.spin(IMU_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    IMU_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
