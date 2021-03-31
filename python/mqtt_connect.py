from wifi_connect import WifiConnect
from passwd_fetch import Password
from umqtt.simple import MQTTClient

# Test reception e.g. with:
# mosquitto_sub -t foo_topic

password_fetch = Password()
passwd = password_fetch.fetch()

print("password:", passwd)

wificonnect = WifiConnect("TPI 2.4g", passwd);
wificonnect.connect()

server = "192.168.1.7"

c = MQTTClient("umqtt_client", server, port = 1883, user = "tiny", password = passwd)
print("mqtt client:", c)

c.connect()
c.publish(b"foo_topic", b"hello")
c.disconnect()
