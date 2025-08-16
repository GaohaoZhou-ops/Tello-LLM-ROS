import rospy
import numpy as np
import time

class MockTello:
    """
    一个模拟 djitellopy.Tello 类的虚拟对象，用于仿真模式。
    它不会进行任何网络通信，只是打印信息并模拟状态。
    """
    def __init__(self):
        self._is_connected = False
        self._stream_on = False
        self._last_cmd_time = time.time()
        rospy.loginfo("Initialized MockTello for simulation.")

    def connect(self):
        rospy.loginfo("[SIM] Connecting to Tello...")
        time.sleep(1)
        self._is_connected = True
        rospy.loginfo("[SIM] Tello connected.")
        return True

    def streamon(self):
        if self._is_connected:
            self._stream_on = True
            rospy.loginfo("[SIM] Video stream ON.")

    def streamoff(self):
        self._stream_on = False
        rospy.loginfo("[SIM] Video stream OFF.")

    def takeoff(self):
        rospy.loginfo("[SIM] Takeoff command sent.")
        time.sleep(2)
        
    def land(self):
        rospy.loginfo("[SIM] Land command sent.")

    def send_rc_control(self, left_right, fwd_back, up_down, yaw):
        if time.time() - self._last_cmd_time > 1.0:
            rospy.loginfo(f"[SIM] RC Control: lr={left_right}, fb={fwd_back}, ud={up_down}, yaw={yaw}")
            self._last_cmd_time = time.time()

    def get_frame_read(self):
        class MockFrame:
            def __init__(self):
                self.frame = np.zeros((720, 960, 3), dtype=np.uint8)
        return MockFrame()
    
    def get_battery(self): return 100
    def get_height(self): return 0
    def get_flight_time(self): return 60
    def get_temperature(self): return 25

    def __getattr__(self, name):
        """
        这个特殊的 Python 方法作为一个“捕获器”。
        如果你尝试调用一个不存在的方法（如 mock.move_forward(...)），
        __getattr__ 会被自动调用，参数'name'就是你尝试访问的方法名（'move_forward'）。
        """
        def generic_handler(*args, **kwargs):
            rospy.loginfo(f"[SIM] Called method '{name}' with arguments: {args}")
        
        if name.startswith(('move_', 'rotate_', 'flip_')):
            return generic_handler
        
        raise AttributeError(f"'MockTello' object has no attribute '{name}'")