import network
import time
import machine

# WiFi configuration
SSID = "vanhoa"
PASSWORD = "11111111"

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    if not wlan.isconnected():
        print("Connecting to WiFi...")
        wlan.connect(SSID, PASSWORD)
        
        # Wait for connection with timeout
        max_wait = 20
        while max_wait > 0:
            if wlan.isconnected():
                break
            max_wait -= 1
            print(".", end="")
            time.sleep(1)
            
    if wlan.isconnected():
        print("\nConnected to WiFi")
        print("IP:", wlan.ifconfig()[0])
        return True
    else:
        print("\nFailed to connect to WiFi")
        return False

# Try to connect to WiFi
for _ in range(3):  # Try 3 times
    if connect_wifi():
        break
    print("Retrying WiFi connection...")
    time.sleep(5)
