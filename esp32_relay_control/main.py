from machine import Pin, I2C
import network
import time
from umqtt.simple import MQTTClient
import random
import ssd1306

# WiFi credentials
SSID = "vanhoa"
PASSWORD = "11111111"

# MQTT Broker settings
MQTT_SERVER = "192.168.137.241"
MQTT_USERNAME = ""
MQTT_PASSWORD = ""
MQTT_PORT = 1883
MQTT_TOPIC_RELAY1 = b"home/relay1"
MQTT_TOPIC_RELAY2 = b"home/relay2"
MQTT_STATUS_RELAY1 = b"home/relay1/status"
MQTT_STATUS_RELAY2 = b"home/relay2/status"

# Pin configuration
RELAY1_PIN = Pin(26, Pin.OUT)  # GPIO26
RELAY2_PIN = Pin(27, Pin.OUT)  # GPIO27
LED1_PIN = Pin(2, Pin.OUT)     # Built-in LED
LED2_PIN = Pin(25, Pin.OUT)    # Changed to GPIO25 for External LED

# State tracking
relay1_state = False
relay2_state = False

def setup_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    print("Kết nối WiFi...")
    
    if not wlan.isconnected():
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep(0.5)
            print(".", end="")
    
    print("\nĐã kết nối WiFi")
    print("IP:", wlan.ifconfig()[0])
    return wlan

def update_relay(pin, led_pin, state, status_topic, new_state):
    global relay1_state, relay2_state
    if status_topic == MQTT_STATUS_RELAY1:
        relay1_state = new_state
    else:
        relay2_state = new_state
    
    pin.value(1 if new_state else 0)
    led_pin.value(1 if new_state else 0)
    client.publish(status_topic, b"ON" if new_state else b"OFF", retain=True)
    print(f"Cập nhật trạng thái: {status_topic.decode()} -> {'ON' if new_state else 'OFF'}")

def callback(topic, msg):
    message = msg.decode()
    new_state = (message == "ON")
    
    if topic == MQTT_TOPIC_RELAY1:
        update_relay(RELAY1_PIN, LED1_PIN, relay1_state, MQTT_STATUS_RELAY1, new_state)
    elif topic == MQTT_TOPIC_RELAY2:
        update_relay(RELAY2_PIN, LED2_PIN, relay2_state, MQTT_STATUS_RELAY2, new_state)

def connect_mqtt():
    client_id = f"ESP32Client-{random.getrandbits(16):04x}"
    client = MQTTClient(client_id, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD)
    client.set_callback(callback)
    
    while True:
        try:
            client.connect()
            print("Kết nối MQTT thành công")
            
            # Subscribe to control topics
            client.subscribe(MQTT_TOPIC_RELAY1)
            client.subscribe(MQTT_TOPIC_RELAY2)
            
            # Publish initial states
            client.publish(MQTT_STATUS_RELAY1, b"OFF", retain=True)
            client.publish(MQTT_STATUS_RELAY2, b"OFF", retain=True)
            break
            
        except Exception as e:
            print("Kết nối thất bại, lỗi =", e)
            print("Thử lại sau 5 giây")
            time.sleep(5)
    
    return client

# Initialize I2C and OLED
try:
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)  # Lower frequency, standard pins
    devices = i2c.scan()
    if devices:
        print("I2C devices found:", [hex(device) for device in devices])
        oled = ssd1306.SSD1306_I2C(128, 64, i2c)
        oled_available = True
    else:
        print("No I2C devices found")
        oled_available = False
except Exception as e:
    print("Error initializing I2C:", e)
    oled_available = False

def update_display():
    if not oled_available:
        return
    oled.fill(0)  # Clear display
    
    # WiFi Status
    oled.text("WiFi: " + ("Connected" if wlan.isconnected() else "Disconnected"), 0, 0)
    if wlan.isconnected():
        oled.text("IP: " + wlan.ifconfig()[0], 0, 10)
    
    # MQTT Status
    try:
        client.ping()
        mqtt_status = "Connected"
    except:
        mqtt_status = "Disconnected"
    oled.text("MQTT: " + mqtt_status, 0, 25)
    
    # Relay States
    oled.text("Relay1: " + ("ON" if relay1_state else "OFF"), 0, 40)
    oled.text("Relay2: " + ("ON" if relay2_state else "OFF"), 0, 50)
    
    oled.show()

# Initialize hardware
RELAY1_PIN.value(0)
RELAY2_PIN.value(0)
LED1_PIN.value(0)
LED2_PIN.value(0)

# Connect to network
wlan = setup_wifi()
client = connect_mqtt()

# Main loop
while True:
    try:
        client.check_msg()
        
        # Update OLED display if available
        if oled_available:
            update_display()
        
        # Visual feedback through built-in LED
        if not wlan.isconnected():
            LED1_PIN.value(1)
            time.sleep(0.1)
            LED1_PIN.value(0)
            time.sleep(0.1)
            
            # Attempt to reconnect WiFi
            wlan = setup_wifi()
            
        time.sleep(0.1)  # Small delay to prevent busy waiting
        
    except Exception as e:
        print("Error:", e)
        # Attempt to reconnect MQTT
        client = connect_mqtt()
