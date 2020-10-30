import rclpy
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import math as math

from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from nav_msgs.msg import *
import matplotlib.pyplot as plt
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Mapping(Node):

    def __init__(self):
        super().__init__('node_mapping')
        self.get_logger().info('Mapping Node Started')
        # Subscriber for LiDAR scan data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        # Subscriber for IMU sensor data
        self.imu_subscription = self.create_subscription(
            Float64,
            'IMU',
            self.imu_callback,
            10)
        # Subscriber for Odometry data  
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        self.imu_subscription
        self.scan_subscription
        self.odom_subscription
  
        self.scanCopied = False
        self.tempScan = LaserScan()					# Laser scan class member to store and process subscribed scan data
        self.bridge = CvBridge()
        self.theta = 0.0
        self.mapSize_X = 900
        self.mapSize_Y = 900
        self.obstaclePixelValue = 1
        self.freeCellPixelValue = 180
        self.unexploredCellValue = 125
        self.robotPositionPixelValue = 250

        self.map = np.full((6,self.mapSize_X,self.mapSize_Y),self.unexploredCellValue,dtype=np.uint8)		# 6 dimensional numpy array. Each array can be used to store different features of the environment
        self.map[2].fill(0)						# Initialise second dimension with zero
        self.map[3].fill(0)						# Initialise third dimension with zero
        self.map2 = np.full((self.mapSize_X,self.mapSize_Y),self.unexploredCellValue,dtype=np.uint8)

    def scan_callback(self, msg):					# Callback function for scan subscriber
        if self.scanCopied is False:					# If scan previous scan data has been already read by odom subscriber, copy new scan data
            self.tempScan.ranges = msg.ranges  
            self.tempScan.intensities = msg.intensities  
            self.scanCopied = True					# Set the flag to indicate the availability of new data
        #self.get_logger().info('Received scan')

    def imu_callback(self, msg):					# Callback function for IMU data subscriber
        self.theta = msg.data						# Copy received IMU value to class memeber variable
        #self.get_logger().info('Received imu')
    
    def odom_callback(self, msg):					# Odom subscriber. The mapping of environment based on received scan,imu and odom happens in this callback method
        #self.get_logger().info('Received odom')
        currentPositionX = msg.pose.pose.position.x			# set current X coordinate of robot from received odom data
        currentPositionY = msg.pose.pose.position.y			# set current Y coordinate of robot from received odom data
   
        x = int((self.mapSize_X/2) + math.ceil(currentPositionX*100))			# The map size is set to 900 centimeters with each cm = 1 pixel. Mapping must start from centre. Hence add offset of 450
        y = int((self.mapSize_Y/2) + math.ceil(currentPositionY*100))
        
        self.map2[x,y] = self.map[0,x,y]

        for i in range(len(self.tempScan.ranges)):			# Iterate through received scan points
            if self.scanCopied is True:					# If new scan data available
                if((self.tempScan.ranges[i]*100 > 35) and (self.tempScan.ranges[i]*100 < 400) and (self.tempScan.intensities[i] > 10)):
			
                    cosA = math.cos(math.radians(i))			# convert lidar scan angle into radians
                    cosB = math.cos(math.radians(self.theta))           # convert current IMU data to radians
                    sinA = math.sin(math.radians(i))
                    sinB = math.sin(math.radians(self.theta))

                    ## Based on 2D translation and rotation (map frame to world frame conversion) calculate the [x,y] coordinates of obstacles using LIDAR scan data and IMU.  ##
                    newX = int(  ( (self.tempScan.ranges[i]*100)*((cosA)*(cosB)) )-( (self.tempScan.ranges[i]*100)*((sinA)*(sinB))) )
                    newY = int(  ( (self.tempScan.ranges[i]*100)*((sinA)*(cosB)) )+( (self.tempScan.ranges[i]*100)*((cosA)*(sinB))) )

                    newX = int(math.ceil(newX/1))			# Resolution of map is 1 => 1 pixel = 1 cm in world coordinates
                    newY = int(math.ceil(newY/1))

                    X = int(newX + x)                  # Add current offset and transation from origin to odom frame (distance travelled by robot) to get exact position of obstacle in world X coordinates 
                    Y = int(newY + y)		       # Add current offset and transation from origin to odom frame (distance travelled by robot) to get exact position of obstacle in world Y coordinates

                   
	
                    if(X < self.mapSize_X and Y < self.mapSize_Y):		# If calculated coordinates are within map size limit	
                        self.map[0,X,Y] = 1
                        if(self.map[1,X,Y] < 10):                               # If pixel value has reduced to a value less than 10 from 125
                            self.map[1,X,Y] = self.obstaclePixelValue           # Mark the pixel value as 1 to indicate the presence of obstacle

                        elif(self.map[1,X,Y] > 10):                             # If pixel value has not reduced to a value less than 10 from 125
                            self.map[1,X,Y] = self.map[1,X,Y] - 3               # Reduce pixel value by 3. This is to make sure that the obstacle is available before marking it in the map
                            self.map2[X,Y] = self.map[1,X,Y]      
              
                        ## Bresenhams Line Algorithm to mark free space and obstacles on map ##
                        dx = (X - x)
                        dy = (Y - y)

                        if(abs(dx) > abs(dy)):
                            steps = abs(dx)
                        else:
                            steps = abs(dy)

                        Xinc = dx /float(steps)
                        Yinc = dy /float(steps)

                        Xnext = x
                        Ynext = y
                        for i in range(steps):
                            self.map2[int(Xnext),int(Ynext)] = self.freeCellPixelValue			# update the pixel as a free space.
                            Xnext += Xinc
                            Ynext += Yinc			
                        
                        self.map2[x,y] = self.robotPositionPixelValue		# update robot position in the map
        try:
            img = self.bridge.cv2_to_imgmsg(self.map2,"mono8")
        except CvBridgeError as e:
            self.get_logger().info(e)
	    #image_pub.publish(img)
        self.scanCopied = False				# reset the flag to fetch new scan data
        cv2.imshow('cv_img2', self.map2)
        cv2.waitKey(2)

def main(args=None):
    rclpy.init(args=args)
    map_publisher = Mapping()
    rclpy.spin(map_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
