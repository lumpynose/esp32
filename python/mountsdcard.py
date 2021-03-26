import machine
import uos
import sdcard

class MountSDCard:
    def __init__(self):
        pass

    def mount(self):
        sd = machine.SDCard(slot = 3, sck = 18, mosi = 23, miso = 38, cs = 4)

        try:
            uos.mount(sd, "/sd")
        except Exception as exc:
            print("Exception:", exc.args[0])

        print("/sd:", uos.listdir("/sd"))
