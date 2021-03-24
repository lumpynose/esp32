from machine import Pin, SoftSPI
import tinypico as TinyPICO
from dotstar import DotStar
import time, random, micropython, gc

# Configure SPI for controlling the DotStar
# Internally we are using software SPI for this as the pins being used are not hardware SPI pins
spi = SoftSPI(sck=Pin( TinyPICO.DOTSTAR_CLK ), mosi=Pin( TinyPICO.DOTSTAR_DATA ), miso=Pin( TinyPICO.SPI_MISO) ) 

# Create a DotStar instance
dotstar = DotStar(spi, 1, brightness = 0.5 ) # Just one DotStar, half brightness

# Turn on the power to the DotStar
TinyPICO.dotstar_power( True )

# Say hello
print("\nHello from TinyPICO!")
print("--------------------\n")

# Show some info on boot 
print("Battery Voltage is {}V".format( TinyPICO.battery_voltage() ) )
print("Battery Charge State is {}\n".format( TinyPICO.battery_charging() ) )

# Show available memory
print("Memory Info - micropython.mem_info()")
print("------------------------------------")
micropython.mem_info()

# Create a colour wheel index int
color_index = 0

# Get the amount of RAM and if it's correct, flash green 3 times before rainbow, otherwise flash red three times
def check_ram():
    gc.collect()
    ram = gc.mem_free()
    col = ( 255, 0, 0, 1 )
    if ram > 4000000:
        col = ( 0, 255, 0, 1 )
    
    for i in range (3):
        dotstar[0] = col
        time.sleep_ms(200)
        dotstar[0] = ( 0, 0, 0, 10 )
        time.sleep_ms(50)
    
    time.sleep_ms(250)

# Check the RAM
check_ram()

# Rainbow colours on the Dotstar
while True:
    # Get the R,G,B values of the next colour
    r,g,b = TinyPICO.dotstar_color_wheel( color_index )
    # Set the colour on the dotstar
    dotstar[0] = ( r, g, b, 0.5)
    # Increase the wheel index
    color_index += 1
    # Sleep for 20ms so the colour cycle isn't too fast
    time.sleep_ms(20)

