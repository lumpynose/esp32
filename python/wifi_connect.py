import network

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

        print('network config:', wlan.ifconfig())
