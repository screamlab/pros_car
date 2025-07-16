import time
from rclpy.node import Node
import rclpy
import curses
import threading
from trajectory_msgs.msg import JointTrajectoryPoint
from pros_car_py.env import *
import numpy as np

STRID_TIME: float = 1.0
JOINT_NUM: int = 13

class BipedKeyboardController(Node):
    def __init__(self, stdscr):
        super().__init__('Biped_keyboard')

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            '/joint_angles',
            10
        )

        self.joint_pos = [0]*JOINT_NUM
        self.standing_pos = self.joint_pos
        self.bac: list[list[float]] = [[0.0]*JOINT_NUM for _ in range(30)]

        self.init_pos = [0,   0,   0,   0,     0,     0,  0,   0,   0,  0,    0,   0, 11]
        self.bac[0]   = [0,  80,   0,   0,     0,    30,  0, -30,   0,  0,    0,   0, 11]
        self.bac[1]   = [0,  80,   0,   0,     0,    30,  0, -30,   0,  0,    0,   0, 12]
        self.bac[2]   = [0,  80,   0,   0,     0,    15,  0, -30,   0, 15,    0,   0, 12]

        self.bac[3]   = [0,  20,   0,   0,     0,    15,  0, -30,   0, 15,    0,   0, 10]
        self.bac[4]   = [0,  80,   0,   0,   -10,    15, 10, -30,   0, 15,    0,   0, 10]
        self.bac[5]   = [0,  30,   0,   0,   -15,    15, 15, -30,   0, 15,    0,   0, 10]
        self.bac[6]   = [0,  15,   0,   0,   -15,    15, 15, -30,   0, 15,    0,   0, 10]
        self.bac[7]   = [0,  -3,   0,   0,   -20,    15, 20, -60,   0, 45,    0,   0, 10]

        self.bac[8]   = [0,  -3,   0,   0,   -25,    15, 25, -60,   0, 45,    0,   0, 10]

        """"
        self.bac[0] = [0.005 ,  -90,   0,   0,     0,     -10,  0,  30,   0,   10,    5,   0] # BAC7
        self.bac[1] = [0.0025,   -90,   0,   0,    10,     -15,  3,  20,  -10,  15,    5,   0] # BAC8
        self.bac[2] = [0     ,   -90,   0,   0,    12,     -20,  5,   5,  -15,  15,    0,   0] # BAC8.1 
        self.bac[3] = [0     ,   -90,   0,   0,    20,     -20,  2,   2,  -30,  30,    0,   0]
        self.bac[4] = [0     ,   -90,   0,   0,    20,     -20,  2,   2,    0,   0,    0,   0]
        """
        """
        self.bac[0] = [ 0.005 ,   0,   0,     0,     -15,  0,  30,    0,   10,    5,   0] # BAC7
        self.bac[1] = [ 0.0025,   0,   0,    10,     -15,  3,  20,  -10,    0,    5,   0] # BAC8
        self.bac[2] = [ 0     ,   0,   0,    15,     -15,  5,   5,    0,    0,    0,   0] # BAC1
        self.bac[3] = [-0.0025,   0,   0,    10,     -10, 10,  10,   20,   -5,    0,   0] # BAC2
        # BAC8.1 

        self.bac[0] = [0.005 ,   0,   0,     0,     -20,  0,  30,   0,   10,    5,   0] # BAC7
        self.bac[1] = [0.0025,   0,   0,    10,     -40,  3,  20,  -10,   0,    5,   0] # BAC8
        self.bac[2] = [0     ,   0,   0,    12,     -50,  5,   5,  -15,  15,    0,   0] # BAC8.1 
        self.bac[3] = [0     ,   0,   0,    20,     -50,  2,   2,  -30,  30,    0,   0]
        self.bac[4] = [0     ,   0,   0,    20,     -50,  2,   2,    0,   0,    0,   0]
        
        """



        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self._car_state_msg = ""
 
    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    if c == ord('w'):
                        self.handle_key_w()
                    elif c == ord('e'):
                        self.handle_key_e()
                    elif c == ord('r'):
                        self.handle_key_r()
                    elif c == ord('t'):
                        self.handle_key_t()
                    elif c == ord('y'):
                        self.handle_key_y()
                    elif c == ord('u'):
                        self.handle_key_u()
                    elif c == ord('b'):
                        self.handle_key_b()
                    elif c == ord('q'):  # Exit on 'q'
                        break
                    self.pub_arm()
                    print()
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.01)

        finally:
            curses.endwin()


    def build_interp_bac(self, bac_src: list[list[float]], n: int) -> list[list[float]]:
        """
        將 bac_src 內的每一對相鄰 key-frame 之間
        插入 n 個線性內插點，回傳展開後的新 list。
        例：len(bac_src)=8、n=3  ⇒  總段數 = 7*(n+1)+1 = 29
        """
        if n < 0:
            raise ValueError("n 必須 >= 0")
        interp_path: list[list[float]] = []

        for i in range(len(bac_src) - 1):
            start = np.array(bac_src[i],  dtype=float)
            end   = np.array(bac_src[i+1],dtype=float)

            # 把起點先收進結果；若不想重複可改成 i==0 時才 append
            interp_path.append(start.tolist())

            # 依比例 t = 1/(n+1), 2/(n+1), …, n/(n+1)
            for k in range(1, n + 1):
                t = k / (n + 1)
                mid = (1 - t) * start + t * end
                interp_path.append(mid.tolist())

        # 把最後一個 key-frame 補上
        interp_path.append(bac_src[-1])

        return interp_path

    def print_basic_info(self, key):
        self.stdscr.clear()
        self.stdscr.move(0, 0)
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"Received msg : {self._car_state_msg}")


    def handle_key_w(self):
        self.interpolate_and_publish(self.init_pos, self.bac[0])
        self.interpolate_and_publish(self.bac[0], self.bac[1])
        self.interpolate_and_publish(self.bac[1], self.bac[2])
        self.interpolate_and_publish(self.bac[2], self.bac[3])

        self.interpolate_and_publish(self.bac[3], self.bac[4])
        self.interpolate_and_publish(self.bac[4], self.bac[5])
        self.interpolate_and_publish(self.bac[5], self.bac[6])
        self.interpolate_and_publish(self.bac[6], self.bac[7])

    def handle_key_e(self):
        self.interpolate_and_publish(self.bac[7], self.bac[8])

    def handle_key_r(self):
        pass

    def handle_key_t(self):
        pass

    def handle_key_y(self):
        pass

    def handle_key_u(self):
        pass

    def handle_key_b(self):
        self.stdscr.addstr(f"biped return to origin state...")
        self.joint_pos = self.standing_pos
        self.pub_arm()

    def pub_arm(self):
        msg = JointTrajectoryPoint()
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0] * len(msg.positions)  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)


    def interpolate_and_publish(self, start, end, step=0.5, delay=0.02):
        current = start.copy()
        steps_required = [
            int(abs(end[i] - start[i]) / step)
            if i != 0 else 0  # index 0 不內插
            for i in range(len(start))
        ]
        max_steps = max(steps_required)

        for s in range(1, max_steps + 1):
            for i in range(1, len(start)):  # 從 index 1 開始內插
                delta = end[i] - start[i]
                if abs(delta) >= step:
                    increment = step if delta > 0 else -step
                    if abs(delta) < abs(increment * s):
                        current[i] = end[i]
                    else:
                        current[i] = start[i] + increment * s
            self.joint_pos = current.copy()
            self.pub_arm()
            time.sleep(delay)

        # 最後一步確保目標正確（整體）
        self.joint_pos = end.copy()
        self.pub_arm()


def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = BipedKeyboardController(stdscr)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
