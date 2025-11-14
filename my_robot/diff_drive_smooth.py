#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class DiffDriveSmooth(Node):
    def __init__(self):
        super().__init__('diff_drive_smooth')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.scan = None

        # سرعات محافظة (عدّلي على راحتك)
        self.v_nom = 0.12       # m/s
        self.w_turn = 0.7       # rad/s
        self.stop_dist = 0.55   # م: ادخُل لفّة لو العائق أقرب من هيك
        self.clear_dist = 0.75  # م: ارجع قدّام لما يبتعد العائق لهون

        self.turn_time = 0.6    # ثواني: كم نثبت على اللفّة قبل إعادة التقييم
        self.state = 'FWD'
        self.turn_until = 0.0
        self.last_cmd = Twist() # للتنعيم
        self.alpha = 0.25       # معامل التنعيم (0..1) الأكبر = أسرع

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    @staticmethod
    def sector_min(ranges, start, end, default):
        vals = []
        n = len(ranges)
        for i in range(start, end):
            r = ranges[i % n]
            if math.isfinite(r):
                vals.append(r)
        if not vals:
            return default
        # median مقاوم للضجيج أكثر من min
        vals.sort()
        return vals[len(vals)//2]

    def tick(self):
        cmd = Twist()
        if self.scan is None:
            self.pub.publish(cmd); return

        r = self.scan.ranges
        n = len(r)
        maxr = self.scan.range_max if self.scan.range_max > 0 else 10.0

        # قطاعات: أمام (±15°)، يسار (45..90°)، يمين (-90..-45°)
        front = self.sector_min(r, n//2 - 15, n//2 + 15, maxr)
        left  = self.sector_min(r, n//2 + 45, n//2 + 90, maxr)
        right = self.sector_min(r, n//2 - 90, n//2 - 45, maxr)

        now = time.time()

        if self.state == 'FWD':
            if front < self.stop_dist:
                # ادخل حالة اللفّ واختر الجهة الأوسع
                self.state = 'TURN'
                self.turn_until = now + self.turn_time
                self.turn_dir = 1.0 if left > right else -1.0
            else:
                cmd.linear.x = self.v_nom
                # تصحيح خفيف بعيداً عن الأقرب
                bias = (1.0/(right+0.1)) - (1.0/(left+0.1))
                cmd.angular.z = max(-0.4, min(0.4, 0.3*bias))

        if self.state == 'TURN':
            cmd.linear.x = 0.0
            cmd.angular.z = self.w_turn * self.turn_dir
            if now >= self.turn_until and front > self.clear_dist:
                self.state = 'FWD'

        # تنعيم الأوامر لتقليل الهزّ
        cmd.linear.x  = self.alpha*cmd.linear.x  + (1-self.alpha)*self.last_cmd.linear.x
        cmd.angular.z = self.alpha*cmd.angular.z + (1-self.alpha)*self.last_cmd.angular.z
        self.last_cmd = cmd

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = DiffDriveSmooth()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

