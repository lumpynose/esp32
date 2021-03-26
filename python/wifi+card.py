from wifi_connect import WifiConnect
from passwd_fetch import Password
from mountsdcard import MountSDCard
# from umqtt.simple import MQTTClient

# Test reception e.g. with:
# mosquitto_sub -t foo_topic

password_fetch = Password()
passwd = password_fetch.fetch()

print("password:", passwd)

wificonnect = WifiConnect("TPI 2.4g", passwd);
wificonnect.connect()

mountsdcard = MountSDCard()
mountsdcard.mount()
    
# c = MQTTClient("umqtt_client", server, port = 1883, user = "tiny", password = passwd)
# c.connect()
# c.publish(b"foo_topic", b"hello")
# c.disconnect()
