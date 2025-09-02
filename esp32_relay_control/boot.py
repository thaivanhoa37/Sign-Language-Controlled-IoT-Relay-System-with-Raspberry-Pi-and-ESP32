# This file is executed on every boot (including wake-boot from deepsleep)
import esp
esp.osdebug(None)

def do_connect():
    import network
    import webrepl
    webrepl.start()
    
# Uncomment this section if you want to install required libraries
# This only needs to be run once
"""
import upip
upip.install('micropython-umqtt.simple')
"""
