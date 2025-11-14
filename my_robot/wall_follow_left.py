#!/usr/bin/env python3
import math, time, random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

CLAMP = lambda x,lo,hi: max(lo,min(hi,x))

class WallFollowLeft(Node):
    def __init__(self):
        super().__init__('wall_follow_left')
        # Params قابلة للتعديل من CLI
        self.declare_parameter('v_nom', 0.12)
        self.declare_parameter('target', 0.50)     # البعد المطلوب عن الجدار الأيسر
        self.declare_parameter('stop_dist', 0.40)  # توقف/لف إذا في شيء أمام
        self.declare_parameter('clear_dist', 0.60) # ارجع قدام بعد اللف لما تفضى الجبهة
        self.declare_parameter('kp_wall', 1.2)
        self.declare_parameter('w_turn', 0.8)
        self.declare_parameter('turn_time', 0.6)

        self.v_nom     = float(self.get_parameter('v_nom').value)
        self.target    = float(self.get_parameter('target').value)
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.clear_dist= float(self.get_parameter('clear_dist').value)
        self.kp_wall   = float(self.get_parameter('kp_wall').value)
        self.w_turn    = float(self.get_parameter('w_turn').value)
        self.turn_time = float(self.get_parameter('turn_time').value)

        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.scan = None
        self.state = 'FWD'
        self.turn_until = 0.0
        self.last_cmd = Twist()
        self.alpha = 0.25  # تنعيم

    def scan_cb(self, msg): self.scan = msg

    @staticmethod
    def sector_min(ranges, start, end, default):
        vals = []
        n = len(ranges)
        for i in range(start, end):
            r = ranges[i % n]
            if math.isfinite(r): vals.append(r)
        if not vals: return default
        vals.sort()
        return vals[len(vals)//2]   # median ضد الضجيج

    def tick(self):
        cmd = Twist()
        if self.scan is None:
            self.pub.publish(cmd); return

        r = self.scan.ranges
        n = len(r); maxr = self.scan.range_max if self.scan.range_max>0 else 10.0

        # الأمام ±15°، اليسار الأمامي 30..60°، اليسار 70..110°
        front = self.sector_min(r, n//2-15, n//2+15, maxr)
        leftF = self.sector_min(r, n//2+30, n//2+60, maxr)
        left  = self.sector_min(r, n//2+70, n//2+110, maxr)

        now = time.time()
        if self.state == 'FWD':
            if front < self.stop_dist:
                self.state = 'TURN'
                self.turn_until = now + self.turn_time
                self.turn_dir = -1.0  # يمين لنتجنب العائق الأمامي
            else:
                # حافظ مسافة عن الجدار اليسار
                err = self.target - left  # موجب يعني بعيد عن الجدار => الف يسار
                w = CLAMP(self.kp_wall * err, -0.5, 0.5)
                # إذا اليسار الأمامي ضيق، ميّل يمين شوي
                if leftF < self.target*0.9: w = min(w, -0.4)
                cmd.linear.x = self.v_nom
                cmd.angular.z = w

        elif self.state == 'TURN':
            cmd.linear.x = 0.0
            cmd.angular.z = self.w_turn * self.turn_dir
            if now >= self.turn_until and front > self.clear_dist:
                self.state = 'FWD'

        # تنعيم
        cmd.linear.x  = self.alpha*cmd.linear.x  + (1-self.alpha)*self.last_cmd.linear.x
        cmd.angular.z = self.alpha*cmd.angular.z + (1-self.alpha)*self.last_cmd.angular.z
        self.last_cmd = cmd
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(WallFollowLeft())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
