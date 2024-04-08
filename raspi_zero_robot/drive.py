import RPi.GPIO as GPIO
import requests, os, socket
from time import sleep

ip = "192.168.4.1"
port = 1234

while os.system(f"ping -c 1 {ip}") != 0:
#machine.idle()
	sleep(0.05)

# left motor dir
l_motor_dir_pin  = 27
l_motor_pwm_pin  = 13
r_motor_dir_pin = 22
r_motor_pwm_pin = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(l_motor_dir_pin,   GPIO.OUT)
GPIO.setup(l_motor_pwm_pin, GPIO.OUT)

GPIO.setup(r_motor_dir_pin, GPIO.OUT)
GPIO.setup(r_motor_pwm_pin, GPIO.OUT)

GPIO.output(l_motor_dir_pin, GPIO.HIGH)
GPIO.output(r_motor_dir_pin, GPIO.LOW)

l_pwm  = GPIO.PWM(l_motor_pwm_pin,  50)
r_pwm = GPIO.PWM(r_motor_pwm_pin, 50)

l_pwm.start(0)
r_pwm.start(0)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.connect((ip, port))

while True:
	sleep(0.1)
	sock.settimeout(0.1)
	sock.sendto(b'a', 0, (ip, port))
	try:
		data = sock.recv(1024).split()
		GPIO.output(l_motor_dir_pin, 1 ^ int(data[2]))
		GPIO.output(r_motor_dir_pin, int(data[3]))
		l_pwm.ChangeDutyCycle(int(float(data[0])/2.55))
		r_pwm.ChangeDutyCycle(int(float(data[1])/2.55))
	except Exception as e:
		print(e)
		print('err')
		continue
