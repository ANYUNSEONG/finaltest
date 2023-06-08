import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
import random
import datetime

# GPIO 핀 번호 설정
Button = 24
RLED = 23
GLED = 27
YLED = 22
Buzzer = 12
Trig = 13
Echo = 19
scale = 261
list = 4

# MQTT 설정
MQTT_HOST = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60
MQTT_PUB_TOPIC = "mobile/anyun/sensing"
MQTT_SUB_TOPIC = "mobile/anyun/led"

# 변수 초기화
GLED_NUM = random.randint(3, 8)
YLED_NUM = random.randint(3, 8)
is_important_alarm = False
input_alarm_time = None
stop_imp = False

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(Button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RLED, GPIO.OUT)
GPIO.setup(GLED, GPIO.OUT)
GPIO.setup(YLED, GPIO.OUT)
GPIO.setup(Buzzer, GPIO.OUT)
GPIO.setup(Trig, GPIO.OUT)
GPIO.setup(Echo, GPIO.IN)

# MQTT 클라이언트 생성
client = mqtt.Client()

# MQTT 연결 이벤트 핸들러
def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker")
    GPIO.output(RLED, GPIO.LOW)
    GPIO.output(YLED, GPIO.LOW)
    GPIO.output(GLED, GPIO.LOW)
    client.subscribe(MQTT_SUB_TOPIC)
    client.publish(MQTT_PUB_TOPIC, "알람 설정을 시작합니다.")
    client.publish(MQTT_PUB_TOPIC, "중요한 알람이면 1을, 간단한 알람이면 2를 입력해주세요.")

# MQTT 메시지 수신 이벤트 핸들러
def on_message(client, userdata, msg):
    global is_important_alarm
    global input_alarm_time

    message = msg.payload.decode()
    print("Received message: " + message)

    if message == "1" or message == "2":
        if message == "1":
            is_important_alarm = True
            client.publish(MQTT_PUB_TOPIC, "중요한 알람을 설정합니다.")
            client.publish(MQTT_PUB_TOPIC, "알람 시간을 설정해 주세요.")
            
        elif message == "2":
            is_important_alarm = False
            client.publish(MQTT_PUB_TOPIC, "간단한 알람을 설정합니다.")
            client.publish(MQTT_PUB_TOPIC, "알람 시간을 설정해 주세요.")
    else:
        try:
            input_alarm_time = datetime.datetime.strptime(message, "%Y.%m.%d. %H:%M")
            current_datetime = datetime.datetime.now()
            if input_alarm_time >= current_datetime:
                client.publish(MQTT_PUB_TOPIC, "알람이 설정되었습니다.")
            else:
                client.publish(MQTT_PUB_TOPIC, "유효하지 않은 알람 시간입니다.")
        except ValueError:
            client.publish(MQTT_PUB_TOPIC, "올바른 시간 형식이 아닙니다.")

# 초음파 센서를 이용한 움직임 감지 함수
def detect_motion():
    GPIO.output(Trig, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(Trig, GPIO.LOW)

    while GPIO.input(Echo) == 0:
        pulse_start = time.time()

    while GPIO.input(Echo) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return distance

# 중요한 알람 종료 함수
def stop_important_alarm():
    global GLED_NUM
    global YLED_NUM
    time.sleep(0.1)
    GPIO.output(Buzzer, GPIO.LOW)
    GPIO.output(GLED, GPIO.LOW)
    GPIO.output(YLED, GPIO.LOW)

    # 원래의 숫자로 초기화
    GLED_NUM = random.randint(3, 8)
    YLED_NUM = random.randint(3, 8)
    
    #정지용 변수
    stop_imp = True
    return stop_imp

# 버튼 입력 감지 콜백 함수
def button_callback(channel):
    global GLED_NUM
    global YLED_NUM
    global stop_imp

    if is_important_alarm:
        if GLED_NUM > 0 or YLED_NUM > 0:
            if GLED_NUM > 0:
                GLED_NUM -= 1
            if YLED_NUM > 0:
                YLED_NUM -= 1
            if YLED_NUM == 0:
                GPIO.output(YLED, GPIO.HIGH)    
            if GLED_NUM == 0:
                GPIO.output(GLED, GPIO.HIGH)
        if GLED_NUM == 0 and YLED_NUM == 0:
            stop_imp = stop_important_alarm()
            

# 버튼 입력 감시 설정
GPIO.setup(Button, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(Button, GPIO.RISING, callback=button_callback, bouncetime=300)

# MQTT 이벤트 핸들러 등록
client.on_connect = on_connect
client.on_message = on_message

# MQTT 브로커에 연결
client.connect(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE)

#Buzzer 코드 복사함
scale = [ 261, 294, 329, 349, 392, 440, 493, 523 ]

try:
    # MQTT 통신 시작
    client.loop_start()
    p = GPIO.PWM(Buzzer, 100)
    while True:
        if is_important_alarm:
            GPIO.output(RLED, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(RLED, GPIO.LOW)
            time.sleep(1)
            if input_alarm_time and datetime.datetime.now() >= input_alarm_time:
                p.start(100)
                p.ChangeDutyCycle(90)
                p.ChangeFrequency(scale[0])
                print("ringring")
                time.sleep(0.5)
                p.ChangeFrequency(scale[0])
                print("ringring")
                time.sleep(0.5)
                
            if stop_imp:
                time.sleep(3)
                GPIO.output(RLED,GPIO.LOW)
                p.stop()
                break            
                    
        else:
            GPIO.output(RLED, GPIO.HIGH)
            time.sleep(1)
            GPIO.output(RLED, GPIO.LOW)
            time.sleep(1)

            if input_alarm_time and datetime.datetime.now() > input_alarm_time:
                distance = detect_motion()
                if distance >= 5:
                    p.start(100)
                    p.ChangeDutyCycle(90)
                    p.ChangeFrequency(261)
                    time.sleep(0.5)
                    p.ChangeFrequency(261)
                    time.sleep(0.5)
                else:
                    client.publish(MQTT_PUB_TOPIC, "간단한 알람이 종료됩니다.")
                    p.stop()
                    break

        time.sleep(1)

except KeyboardInterrupt:
    pass

finally:
    # GPIO 리소스 해제
    GPIO.cleanup()
    
    # MQTT 통신 종료
    client.loop_stop()
    client.disconnect()