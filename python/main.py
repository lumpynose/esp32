from wifi_connect import WifiConnect
from umqtt.simple import MQTTClient
from passwd_fetch import Password

# Test reception e.g. with:
# mosquitto_sub -t foo_topic

def main(server = "192.168.1.7"):
    password_fetch = Password()
    passwd = password_fetch.fetch()

    print("password:", passwd)

    wificonnect = WifiConnect("The Pulsating Inconvenience", passwd);
    wificonnect.connect()
    
    c = MQTTClient("umqtt_client", server, port = 1883, user = "tiny", password = passwd)
    c.connect()
    c.publish(b"foo_topic", b"hello")
    c.disconnect()

if __name__ == "__main__":
    main()
