import os
import curses
import threading
import rclpy
import time
from pros_car_py.joint_config import JOINT_UPDATES_POSITIVE, JOINT_UPDATES_NEGATIVE
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller import ArmController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator
from pros_car_py.custom_control import CustomControl
from pros_car_py.ik_solver import RobotIKSolver
import io
import sys

class KeyboardController:
    """鍵盤控制邏輯，專注於定義按鍵與控制行為的對應"""

    def __init__(self, stdscr, car_controller, arm_controller, custom_control, default_vel=10):
        self.stdscr = stdscr
        self.last_key = None
        self.car_controller = car_controller
        self.arm_controller = arm_controller
        self.custom_control = custom_control
        self.vel = float(os.getenv("WHEEL_SPEED", default_vel))

        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)

        self.mode = None
        self.output_buffer = io.StringIO()
        self.auto_nav_thread = None
        self.auto_nav_running = False
        self.auto_arm_thread = None
        self.auto_arm_running = False

    def run_mode(self):
        """在特定模式下運行鍵盤控制邏輯"""
        self.stdscr.clear()
        self.stdscr.nodelay(True)  # 設置為非阻塞模式
        if self.mode == "Auto Nav":
            self.start_auto_nav()
        elif self.mode == "Auto Arm Control":
            self.start_auto_arm()
        
        while True:
            if self.mode in ["Car Control", "Arm Control"]:
                self.display_mode_info()
            else:
                self.display_other_mode_info()
            
            c = self.stdscr.getch()
            if c == ord("q"):
                if self.auto_nav_thread:
                    self.stop_auto_nav()
                if self.auto_arm_thread:
                    self.stop_auto_arm()
                break
            elif c != -1:
                self.handle_key_input(chr(c))
            
            time.sleep(0.1)  # 短暫休眠以減少 CPU 使用率

    def display_mode_info(self):
        """顯示當前模式的信息和控制說明"""
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, f"Current mode: {self.mode}")
        self.stdscr.addstr(1, 0, "Controls:")
        # 這裡可以根據不同的模式顯示不同的控制說明
        if self.mode == "Car Control":
            self.stdscr.addstr(2, 0, "Use arrow keys to control the car")
        elif self.mode == "Arm Control":
            self.stdscr.addstr(2, 0, "Use number keys to control arm joints")
        elif self.mode == "Combined Control":
            self.stdscr.addstr(2, 0, "Use both car and arm controls")
        elif self.mode == "Custom Control":
            self.stdscr.addstr(2, 0, "Use custom control keys")
        self.stdscr.addstr(4, 0, "Press 'q' to return to main menu")
        if self.last_key:
            self.stdscr.addstr(6, 0, f"Last key pressed: {self.last_key}")
        self.stdscr.refresh()

    def display_other_mode_info(self):
        """顯示其他模式的內部打印信息"""
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, f"Current mode: {self.mode}")
        self.stdscr.addstr(1, 0, "Internal print output:")
        y = 2
        for line in self.output_buffer.getvalue().split('\n'):
            self.stdscr.addstr(y, 0, line[:curses.COLS-1])  # 確保不超過屏幕寬度
            y += 1
            if y >= curses.LINES - 1:  # 確保不超過屏幕高度
                break
        self.stdscr.addstr(curses.LINES-1, 0, "Press 'q' to return to main menu")
        self.stdscr.refresh()

    def handle_key_input(self, c):
        """處理鍵盤輸入"""
        old_stdout = sys.stdout
        sys.stdout = self.output_buffer
        try:
            if self.mode == "Car Control":
                self.car_controller.manual_control(c)
            elif self.mode == "Arm Control":
                self.arm_controller.manual_control(c)
            elif self.mode == "Auto Arm Control":
                self.arm_controller.auto_control()
            elif self.mode == "Custom Control":
                self.custom_control.manual_control(c)
            self.last_key = c
        finally:
            sys.stdout = old_stdout
        if self.mode in ["Car Control", "Arm Control"]:
            self.display_mode_info()
        else:
            self.display_other_mode_info()

    def start_auto_nav(self):
        if not self.auto_nav_thread:
            self.auto_nav_running = True
            self.auto_nav_thread = threading.Thread(target=self.run_auto_nav)
            self.auto_nav_thread.start()

    def stop_auto_nav(self):
        if self.auto_nav_thread:
            self.auto_nav_running = False
            self.auto_nav_thread.join()
            self.auto_nav_thread = None

    def run_auto_nav(self):
        while self.auto_nav_running and rclpy.ok():
            old_stdout = sys.stdout
            sys.stdout = self.output_buffer
            try:
                self.car_controller.auto_control("auto_nav")
            finally:
                sys.stdout = old_stdout
            self.display_other_mode_info()
            time.sleep(0.1)  # 控制更新頻率

    def start_auto_arm(self):
        if not self.auto_arm_thread:
            self.auto_arm_running = True
            self.auto_arm_thread = threading.Thread(target=self.run_auto_arm)
            self.auto_arm_thread.start()

    def stop_auto_arm(self):
        if self.auto_arm_thread:
            self.auto_arm_running = False
            self.auto_arm_thread.join()
            self.auto_arm_thread = None

    def run_auto_arm(self):
        while self.auto_arm_running and rclpy.ok():
            old_stdout = sys.stdout
            sys.stdout = self.output_buffer
            try:
                self.arm_controller.auto_control()
            finally:
                sys.stdout = old_stdout
            self.display_other_mode_info()
            time.sleep(0.1)  # 控制更新頻率

def init_ros_node():
    rclpy.init()
    node = RosCommunicator()
    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()
    return node, thread

def main():
    stdscr = curses.initscr()
    ros_communicator, ros_thread = init_ros_node()
    data_processor = DataProcessor(ros_communicator)
    nav2_processing = Nav2Processing(ros_communicator, data_processor)
    car_controller = CarController(ros_communicator, nav2_processing)
    ik_solver = RobotIKSolver()
    arm_controller = ArmController(ros_communicator, nav2_processing, ik_solver, num_joints = 5)
    custom_control = CustomControl(car_controller, arm_controller)
    keyboard_controller = KeyboardController(stdscr, car_controller, arm_controller, custom_control, default_vel=10)
    
    while True:
        try:
            while True:
                stdscr.clear()
                stdscr.addstr(0, 0, "Select a mode:")
                stdscr.addstr(1, 0, "1. Car Control")
                stdscr.addstr(2, 0, "2. Arm Control")
                stdscr.addstr(3, 0, "3. Auto Nav")
                stdscr.addstr(4, 0, "4. Auto Arm Control")
                stdscr.addstr(5, 0, "5. Custom Control")
                stdscr.addstr(6, 0, "q. Quit")
                stdscr.refresh()

                choice = stdscr.getch()
                if choice == ord('q'):
                    break
                elif choice in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                    mode = {ord('1'): "Car Control", ord('2'): "Arm Control", ord('3'): "Auto Nav", ord('4'): "Auto Arm Control", ord('5'): "Custom Control"}[choice]
                    keyboard_controller.mode = mode
                    keyboard_controller.output_buffer = io.StringIO()  # 清空輸出緩衝區
                    keyboard_controller.run_mode()
        finally:
            curses.endwin()
            rclpy.shutdown()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()
