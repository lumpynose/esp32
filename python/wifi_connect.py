import network
import machine

class WifiConnect:
    def __init__(self, essid, password):
        self.essid = essid
        self.password = password
        
    def connect(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)

        if not wlan.isconnected():
            print('connecting to network...')

            wlan.connect(self.essid, self.password)

            while not wlan.isconnected():
                pass
                #print(wlan.status())

        print('network config:', wlan.ifconfig())
        print('MAC:', wlan.config('mac'))
        print('machine id:',  machine.unique_id())
