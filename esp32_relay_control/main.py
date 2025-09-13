from machine import Pin
import network
import time
from umqtt.robust import MQTTClient
import random
import gc

# WiFi credentials
SSID = "vanhoa"
PASSWORD = "11111111"

# MQTT Broker settings
MQTT_SERVER = "192.168.137.127"  # Pi's IP address
MQTT_PORT = 1883
MQTT_KEEPALIVE = 30

# MQTT Topics
MQTT_TOPIC_RELAY1 = b"home/relay1"
MQTT_TOPIC_RELAY2 = b"home/relay2"
MQTT_STATUS_RELAY1 = b"home/relay1/status"
MQTT_STATUS_RELAY2 = b"home/relay2/status"
MQTT_LWT_TOPIC = b"home/esp32/status"

# Pin configuration
RELAY1_PIN = 26  # GPIO26
RELAY2_PIN = 27  # GPIO27
LED1_PIN = 2     # Built-in LED
LED2_PIN = 4     # External LED

# Initialize pins
relay1 = Pin(RELAY1_PIN, Pin.OUT)
relay2 = Pin(RELAY2_PIN, Pin.OUT)
led1 = Pin(LED1_PIN, Pin.OUT)
led2 = Pin(LED2_PIN, Pin.OUT)

# Global variables
client = None
last_mqtt_check = 0
MQTT_CHECK_INTERVAL = 5000  # 5 seconds
mqtt_connected = False

# Initialize state
relay1_state = False
relay2_state = False

def blink_led(led, times=1):
    """Blink LED to indicate status"""
    for _ in range(times):
        led.value(1)
        time.sleep_ms(100)
        led.value(0)
        time.sleep_ms(100)

def get_wifi_status():
    """Get WiFi connection status and IP"""
    wlan = network.WLAN(network.STA_IF)
    if wlan.isconnected():
        return True, wlan.ifconfig()[0]
    return False, None

def ensure_wifi_connected():
    """Ensure WiFi is connected before proceeding"""
    wifi_connected, ip = get_wifi_status()
    if not wifi_connected:
        print("WiFi not connected. Please check boot.py configuration.")
        blink_led(led1, 5)
        return False
    print(f"WiFi connected, IP: {ip}")
    return True

def create_mqtt_client():
    """Create and configure MQTT client"""
    global client
    client_id = f"ESP32Client-{random.getrandbits(16):04x}"
    
    try:
        client = MQTTClient(client_id, MQTT_SERVER,
                          port=MQTT_PORT,
                          keepalive=MQTT_KEEPALIVE)
        
        client.set_last_will(MQTT_LWT_TOPIC, b"offline", retain=True)
        client.set_callback(mqtt_callback)
        
        return client
    except Exception as e:
        print(f"Error creating MQTT client: {e}")
        return None

def mqtt_callback(topic, msg):
    """Handle incoming MQTT messages"""
    try:
        topic = topic.decode('utf-8')
        msg = msg.decode('utf-8')
        print(f"Received message on {topic}: {msg}")
        
        new_state = (msg == "ON")
        
        if topic == MQTT_TOPIC_RELAY1.decode():
            update_relay(relay1, led1, "relay1_state", MQTT_STATUS_RELAY1, new_state)
            blink_led(led2)
        elif topic == MQTT_TOPIC_RELAY2.decode():
            update_relay(relay2, led2, "relay2_state", MQTT_STATUS_RELAY2, new_state)
            blink_led(led2)
    except Exception as e:
        print(f"Error in callback: {e}")
        blink_led(led1, 3)

def update_relay(pin, led_pin, state_var, status_topic, new_state):
    """Update relay state and publish status"""
    global relay1_state, relay2_state
    
    try:
        # Update state
        if state_var == "relay1_state":
            relay1_state = new_state
        else:
            relay2_state = new_state
        
        # Set pin values
        pin.value(1 if new_state else 0)
        led_pin.value(1 if new_state else 0)
        
        # Publish status
        if client and mqtt_connected:
            client.publish(status_topic, b"ON" if new_state else b"OFF", retain=True)
            print(f"Published status: {status_topic.decode()} -> {'ON' if new_state else 'OFF'}")
    except Exception as e:
        print(f"Error updating relay: {e}")
        blink_led(led1, 3)

def connect_mqtt():
    """Connect to MQTT broker with error handling"""
    global client, mqtt_connected
    
    # Check WiFi first
    if not ensure_wifi_connected():
        return False
    
    try:
        print(f"Connecting to MQTT broker at {MQTT_SERVER}...")
        
        if client:
            try:
                client.disconnect()
            except:
                pass
        
        client = create_mqtt_client()
        if not client:
            return False
        
        client.connect()
        print("MQTT Connected")
        
        # Subscribe to topics
        client.subscribe(MQTT_TOPIC_RELAY1)
        client.subscribe(MQTT_TOPIC_RELAY2)
        
        # Publish online status
        client.publish(MQTT_LWT_TOPIC, b"online", retain=True)
        
        # Publish initial states
        client.publish(MQTT_STATUS_RELAY1, b"OFF" if not relay1_state else b"ON", retain=True)
        client.publish(MQTT_STATUS_RELAY2, b"OFF" if not relay2_state else b"ON", retain=True)
        
        mqtt_connected = True
        blink_led(led2, 2)
        return True
        
    except Exception as e:
        mqtt_connected = False
        print(f"MQTT Connection failed: {e}")
        blink_led(led1, 3)
        return False

def main():
    """Main program loop"""
    global client, mqtt_connected, last_mqtt_check
    
    print("Initializing...")
    
    # Initialize hardware
    relay1.value(0)
    relay2.value(0)
    led1.value(0)
    led2.value(0)
    
    # Ensure WiFi is connected
    if not ensure_wifi_connected():
        print("Cannot proceed without WiFi connection")
        while True:
            blink_led(led1, 2)
            time.sleep(1)
    
    print("WiFi connection verified")
    time.sleep(1)  # Wait a bit before MQTT connection
    
    # Initial MQTT connection
    while not connect_mqtt():
        print("Retrying MQTT connection in 5 seconds...")
        time.sleep(5)
    
    print("Setup complete, entering main loop")
    
    while True:
        try:
            current_time = time.ticks_ms()
            
            # Check WiFi and MQTT connection status periodically
            if time.ticks_diff(current_time, last_mqtt_check) >= MQTT_CHECK_INTERVAL:
                wifi_connected, _ = get_wifi_status()
                
                if not wifi_connected:
                    print("WiFi disconnected!")
                    mqtt_connected = False
                    blink_led(led1)
                elif not mqtt_connected:
                    print("Attempting MQTT reconnection...")
                    connect_mqtt()
                
                last_mqtt_check = current_time
            
            if mqtt_connected:
                # Check for MQTT messages
                client.check_msg()
                
                # Memory management
                gc.collect()
            
            time.sleep_ms(100)
            
        except Exception as e:
            print(f"Error in main loop: {e}")
            mqtt_connected = False
            blink_led(led1, 3)
            time.sleep(5)

if __name__ == "__main__":
    main()
