import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Motor Pins (BCM numbering)
# Drive motor
IN1, IN2, ENA = 17, 27, 18
# Steering motor
IN3, IN4, ENB = 22, 23, 25

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

# Create PWM objects (100 Hz)
pwm_drive = GPIO.PWM(ENA, 100)
pwm_steer = GPIO.PWM(ENB, 100)
pwm_drive.start(0)
pwm_steer.start(0)

class AckermannMotorController(Node):
    def __init__(self):
        super().__init__('ackermann_motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info("Motor controller initialized")

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        self.get_logger().info(f"cmd_vel: linear={linear:.2f}, angular={angular:.2f}")

        # Drive control
        if linear > 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        elif linear < 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)

        pwm_drive.ChangeDutyCycle(min(abs(linear) * 100, 100))

        # Steering control
        if angular > 0:
            GPIO.output(IN3, GPIO.HIGH)
            GPIO.output(IN4, GPIO.LOW)
        elif angular < 0:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.HIGH)
        else:
            GPIO.output(IN3, GPIO.LOW)
            GPIO.output(IN4, GPIO.LOW)

        pwm_steer.ChangeDutyCycle(min(abs(angular) * 100, 100))

    def destroy_node(self):
        pwm_drive.stop()
        pwm_steer.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AckermannMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
