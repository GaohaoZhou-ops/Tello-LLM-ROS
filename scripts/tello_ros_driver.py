#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovarianceStamped 
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
import cv2
from cv_bridge import CvBridge
import tf
from math import sin, cos, pi
import numpy as np

try:
    from djitellopy import Tello
except ImportError:
    rospy.logerr("djitellopy not found. Please install it using 'pip install djitellopy'")
    Tello = None

from mock_tello import MockTello
from tello_llm_ros.srv import Move, MoveResponse

class TelloROSNode:
    def __init__(self):
        rospy.init_node('tello_ros_node')

        self.drone_name = rospy.get_param("~drone_name", "tello")
        rospy.loginfo(f"Initializing Tello driver for drone: {self.drone_name}")

        self.odom_frame = f"{self.drone_name}/odom"
        self.base_frame = f"{self.drone_name}/base_link"

        self.use_sim = rospy.get_param("~use_sim", False)
        self.cmd_vel_timeout = rospy.Duration(rospy.get_param("~cmd_vel_timeout", 0.5))
        self.last_cmd_vel_time = rospy.Time.now()

        if self.use_sim:
            rospy.loginfo(f"Running {self.drone_name} in SIMULATION mode.")
            self.tello = MockTello()
        else:
            rospy.loginfo(f"Running {self.drone_name} in REAL mode.")
            if Tello is None:
                rospy.logfatal("Cannot run in real mode, djitellopy library is missing.")
                return
            self.tello = Tello()

        self.tello.connect()
        if not self.use_sim:
            rospy.loginfo(f"{self.drone_name} battery: {self.tello.get_battery()}%")
        self.tello.streamon()
        
        self.bridge = CvBridge()

        # --- 发布器、订阅器和服务的名称现在都使用 drone_name ---
        self.image_pub = rospy.Publisher(f'{self.drone_name}/image_raw', Image, queue_size=10)
        self.status_pub = rospy.Publisher(f'{self.drone_name}/status', String, queue_size=10)
        self.odom_pub = rospy.Publisher(f'{self.drone_name}/odom', Odometry, queue_size=10)
        
        self.tf_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber(f'{self.drone_name}/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)
        
        rospy.Service(f'{self.drone_name}/takeoff', Trigger, self.takeoff_service)
        rospy.Service(f'{self.drone_name}/land', Trigger, self.land_service)
        rospy.Service(f'{self.drone_name}/flip_forward', Trigger, self.flip_forward_service)
        rospy.Service(f'{self.drone_name}/flip_backward', Trigger, self.flip_backward_service)
        rospy.Service(f'{self.drone_name}/flip_left', Trigger, self.flip_left_service)
        rospy.Service(f'{self.drone_name}/flip_right', Trigger, self.flip_right_service)

        rospy.Service(f'{self.drone_name}/move_forward', Move, self._create_move_handler('forward', 'distance'))
        rospy.Service(f'{self.drone_name}/move_backward', Move, self._create_move_handler('back', 'distance'))
        rospy.Service(f'{self.drone_name}/move_left', Move, self._create_move_handler('left', 'distance'))
        rospy.Service(f'{self.drone_name}/move_right', Move, self._create_move_handler('right', 'distance'))
        rospy.Service(f'{self.drone_name}/move_up', Move, self._create_move_handler('up', 'distance'))
        rospy.Service(f'{self.drone_name}/move_down', Move, self._create_move_handler('down', 'distance'))
        rospy.Service(f'{self.drone_name}/rotate_clockwise', Move, self._create_move_handler('clockwise', 'angle'))
        rospy.Service(f'{self.drone_name}/rotate_counter_clockwise', Move, self._create_move_handler('counter_clockwise', 'angle'))
        
        self.x, self.y, self.z, self.yaw = 0.0, 0.0, 0.0, 0.0
        self.current_twist = Twist()
        self.last_update_time = rospy.Time.now()
        
        self.takeoff_height = 0.8
        self.flip_dist = 0.5

        rospy.Timer(rospy.Duration(1.0 / 30.0), self.update_loop)
        rospy.Timer(rospy.Duration(1.0), self.publish_status)

        rospy.loginfo(f"Tello ROS node for {self.drone_name} with watchdog started.")

    def _create_move_handler(self, command, unit):
        """
        Factory to create a ROS service handler for a specific Tello move command.
        """
        def handler(req):
            rospy.loginfo(f"Executing '{command}' with value {req.value}")
            value_in_sdk_units = 0
            
            try:
                if unit == 'distance':
                    value_in_sdk_units = int(req.value * 100)
                    if not (20 <= value_in_sdk_units <= 500):
                        raise ValueError("Distance must be between 0.2 and 5.0 meters.")
                elif unit == 'angle':
                    if req.value > 10:
                        value_in_sdk_units = int(req.value)
                    else:
                        value_in_sdk_units = int(req.value * 180.0 / pi)
                    if not (1 <= value_in_sdk_units <= 360):
                        raise ValueError(f"Angle must be between ~0.017 and ~6.28 radians (1-360 degrees), command is {req.value}")

                tello_function = getattr(self.tello, f"move_{command}" if unit == 'distance' else f"rotate_{command}")
                tello_function(value_in_sdk_units)

                # --- Update Ideal Odometry ---
                dist = req.value # meters or radians
                if command == 'forward':
                    self.x += dist * cos(self.yaw)
                    self.y += dist * sin(self.yaw)
                elif command == 'back':
                    self.x -= dist * cos(self.yaw)
                    self.y -= dist * sin(self.yaw)
                elif command == 'right':
                    self.x += dist * sin(self.yaw)
                    self.y -= dist * cos(self.yaw)
                elif command == 'left':
                    self.x -= dist * sin(self.yaw)
                    self.y += dist * cos(self.yaw)
                elif command == 'up':
                    self.z += dist
                elif command == 'down':
                    self.z -= dist
                elif command == 'counter_clockwise':
                    self.yaw += dist
                elif command == 'clockwise':
                    self.yaw -= dist
                    
                self.last_update_time = rospy.Time.now()

                return MoveResponse(success=True, message=f"Successfully executed '{command}'.")

            except Exception as e:
                rospy.logerr(f"Failed to execute '{command}': {e}")
                return MoveResponse(success=False, message=str(e))
        return handler


    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        if dt <= 0: return

        delta_x = (self.current_twist.linear.x * cos(self.yaw) - self.current_twist.linear.y * sin(self.yaw)) * dt
        delta_y = (self.current_twist.linear.x * sin(self.yaw) + self.current_twist.linear.y * cos(self.yaw)) * dt
        delta_z = self.current_twist.linear.z * dt
        delta_yaw = self.current_twist.angular.z * dt

        self.x += delta_x
        self.y += delta_y
        self.z += delta_z
        self.yaw += delta_yaw

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position = Point(self.x, self.y, self.z)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.twist.twist = self.current_twist
        self.odom_pub.publish(odom)

        self.tf_broadcaster.sendTransform(
            (self.x, self.y, self.z),
            odom_quat,
            current_time,
            self.base_frame,
            self.odom_frame
        )
        self.last_update_time = current_time

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = rospy.Time.now()
        self.current_twist = msg
        self.tello.send_rc_control(
            int(-msg.linear.y * 100),
            int(msg.linear.x * 100),
            int(msg.linear.z * 100),
            int(-msg.angular.z * 100)
        )
    
    def initial_pose_callback(self, msg):
        """
        当收到来自 RViz 的 '2D Pose Estimate' 时，重置里程计状态.
        """
        pose = msg.pose.pose
        
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # 更新无人机的状态
        # 注意：只更新 x, y 和 yaw，因为这是 "2D" 位姿估计。
        # 保持当前的 z (高度) 不变，因为 RViz 的2D估计通常不包含准确的高度信息。
        self.x = pose.position.x
        self.y = pose.position.y
        self.yaw = yaw
        
        self.current_twist = Twist()
        self.last_update_time = rospy.Time.now()

        rospy.loginfo(f"[{self.drone_name}] Pose reset by RViz. New pose: x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f}")

    
    def update_loop(self, event):
        if rospy.Time.now() - self.last_cmd_vel_time > self.cmd_vel_timeout:
            if self.current_twist != Twist():
                rospy.loginfo(f"[{self.drone_name}] cmd_vel timeout. Halting drone.")
                self.current_twist = Twist()
                self.tello.send_rc_control(0, 0, 0, 0)
        
        frame = self.tello.get_frame_read().frame
        if frame is not None:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "rgb8")
                ros_image.header.stamp = rospy.Time.now()
                self.image_pub.publish(ros_image)
            except Exception as e:
                rospy.logerr(f"Error publishing image: {e}")
        
        self.update_odometry()

    def publish_status(self, event):
        try:
            status_msg = String()
            status_msg.data = f"Battery: {self.tello.get_battery()}%, Height: {self.tello.get_height()}cm, " \
                             f"Flight Time: {self.tello.get_flight_time()}s, " \
                             f"Temperature: {self.tello.get_temperature()}C"
            self.status_pub.publish(status_msg)
        except Exception as e:
            rospy.logerr(f"Could not get Tello status: {e}")

    def takeoff_service(self, req):
        try:
            self.tello.takeoff()
            self.z = self.takeoff_height
            self.last_update_time = rospy.Time.now()
            return TriggerResponse(success=True, message="Takeoff successful")
        except Exception as e:
            rospy.logerr(f"Takeoff failed: {e}")
            return TriggerResponse(success=False, message=str(e))

    def land_service(self, req):
        try:
            self.tello.land()
            self.x, self.y, self.z, self.yaw = 0.0, 0.0, 0.0, 0.0
            self.current_twist = Twist()
            self.last_update_time = rospy.Time.now()
            return TriggerResponse(success=True, message="Land successful")
        except Exception as e:
            rospy.logerr(f"Land failed: {e}")
            return TriggerResponse(success=False, message=str(e))
    
    def _create_flip_service_handler(self, direction):
        def handler(req):
            try:
                flip_func_name = f"flip_{direction}"
                if hasattr(self.tello, flip_func_name):
                    getattr(self.tello, flip_func_name)()
                else: 
                    rospy.loginfo(f"[{self.drone_name}/SIM] Executing flip_{direction}")

                if direction == 'forward':
                    self.x += self.flip_dist * cos(self.yaw)
                    self.y += self.flip_dist * sin(self.yaw)
                elif direction == 'back':
                    self.x -= self.flip_dist * cos(self.yaw)
                    self.y -= self.flip_dist * sin(self.yaw)
                elif direction == 'left':
                    self.x += self.flip_dist * cos(self.yaw - pi/2)
                    self.y += self.flip_dist * sin(self.yaw - pi/2)
                elif direction == 'right':
                    self.x += self.flip_dist * cos(self.yaw + pi/2)
                    self.y += self.flip_dist * sin(self.yaw + pi/2)

                self.last_update_time = rospy.Time.now()
                return TriggerResponse(success=True, message=f"Flip {direction} successful")
            except Exception as e:
                rospy.logerr(f"Flip {direction} failed: {e}")
                return TriggerResponse(success=False, message=str(e))
        return handler
        
    @property
    def flip_forward_service(self): return self._create_flip_service_handler('forward')
    
    @property
    def flip_backward_service(self): return self._create_flip_service_handler('back')

    @property
    def flip_left_service(self): return self._create_flip_service_handler('left')
    
    @property
    def flip_right_service(self): return self._create_flip_service_handler('right')

    def on_shutdown(self):
        rospy.loginfo(f"Shutting down Tello ROS node for {self.drone_name}.")
        try:
            self.tello.land()
        except Exception as e:
            rospy.logerr(f"Error on landing during shutdown: {e}")
        try:
            self.tello.streamoff()
        except Exception as e:
            rospy.logerr(f"Error on streamoff during shutdown: {e}")


if __name__ == '__main__':
    try:
        node = TelloROSNode()
        rospy.on_shutdown(node.on_shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"[FATAL] An unhandled exception occurred in main: {e}")