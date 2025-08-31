#!/usr/bin python3.10

# Write your code here

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.time import Time
import matplotlib.pyplot as plt
import csv

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.publisher_ = self.create_publisher(Float32, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/current_speed',
            self.speed_callback,
            10)
        
        # PID gains (tuned for fast settling <8s, low overshoot <10%, steady error <0.3)
        self.kp = 0.5
        self.ki = 0.05
        self.kd = 0.1
        
        # Variables
        self.target_speed = 0.0  # Starts at 0, changes over time for extra credit
        self.vvs_integral = 0.0  # Integral with 'vvs_' prefix
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Data collection for plotting and CSV
        self.times = []
        self.actual_speeds = []
        self.target_speeds = []
        self.start_time = self.get_clock().now()
        
    def get_variable_target(self, elapsed):
        if elapsed < 10.0:
            return 0.0
        elif elapsed < 40.0:
            return 60.0
        elif elapsed < 70.0:
            return 30.0
        else:
            return 50.0
        
    def speed_callback(self, msg):
        current_time = self.get_clock().now()
        dt_nanoseconds = (current_time - self.prev_time).nanoseconds
        dt = dt_nanoseconds / 1e9 if dt_nanoseconds > 0 else 0.05  # Fallback to 50ms
        
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        self.target_speed = self.get_variable_target(elapsed)
        
        error = self.target_speed - msg.data
        self.vvs_integral += error * dt
        derivative = (error - self.prev_error) / dt
        
        control = self.kp * error + self.ki * self.vvs_integral + self.kd * derivative
        clamped_control = max(min(control, 1.0), -1.0)
        
        # Anti-windup: Stop integrating if clamped and error would increase saturation
        if (clamped_control >= 1.0 and error > 0) or (clamped_control <= -1.0 and error < 0):
            self.vvs_integral -= error * dt
        
        cmd_msg = Float32()
        cmd_msg.data = clamped_control
        self.publisher_.publish(cmd_msg)
        
        # Log to terminal
        self.get_logger().info(f'Elapsed: {elapsed:.2f}s, Current speed: {msg.data:.2f} km/h, Target: {self.target_speed:.2f} km/h, Control: {clamped_control:.2f}')
        
        # Collect data
        self.times.append(elapsed)
        self.actual_speeds.append(msg.data)
        self.target_speeds.append(self.target_speed)
        
        self.prev_error = error
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save plot
        plt.figure()
        plt.plot(node.times, node.actual_speeds, label='Actual Speed')
        plt.plot(node.times, node.target_speeds, label='Target Speed')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed (km/h)')
        plt.title('Vehicle Speed Control')
        plt.legend()
        plt.grid(True)
        plt.savefig('speed_plot.png')
        node.get_logger().info('Saved plot to speed_plot.png')
        
        # Save CSV
        with open('speed_data.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Time (s)', 'Actual Speed (km/h)', 'Target Speed (km/h)'])
            for t, actual, target in zip(node.times, node.actual_speeds, node.target_speeds):
                writer.writerow([f'{t:.2f}', f'{actual:.2f}', f'{target:.2f}'])
        node.get_logger().info('Saved data to speed_data.csv')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
