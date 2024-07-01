#!/usr/bin/python3

import rospy
from std_msgs.msg import Int32

# Importa a biblioteca RPi.GPIO para controle de GPIO
import RPi.GPIO as GPIO

# Define a porta GPIO utilizada para controlar o servo
SERVO_PIN = 6

# Inicializa o modo GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Cria um objeto PWM para controlar o servo
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # Frequência de PWM de 50 Hz

def servo_position_callback(msg):
    # Converte o valor recebido (de 0 a 90) para a posição do servo
    servo_position = msg.data
    duty_cycle = servo_position / 18 + 2  # Converte graus para duty cycle (ajuste conforme necessário)
    servo_pwm.start(duty_cycle)

def servo_control_node():
    rospy.init_node('servo_control_node', anonymous=True)
    rospy.Subscriber('servo_position', Int32, servo_position_callback)
    rospy.spin()
    


if __name__ == '__main__':
    try:
        servo_control_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()  # Limpa o estado dos GPIOs ao finalizar


