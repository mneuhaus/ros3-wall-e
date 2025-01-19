import time
import math
from plasma import WS2812
from servo import Servo, servo2040

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

# Set first LED to green to indicate power
led_bar.set_rgb(0, 0, 255, 0)

# Create a servo on pin 0
s = Servo(servo2040.SERVO_1)

# Main loop
try:
    # Enable the servo
    s.enable()
    
    while True:
        # Go to min
        s.to_min()
        time.sleep(2)
        
        # Go to max
        s.to_max()
        time.sleep(2)
        
        # Go back to mid
        s.to_mid()
        time.sleep(2)
        
        # Do a sine sweep
        for i in range(360):
            s.value(math.sin(math.radians(i)) * 90.0)
            time.sleep(0.02)

except KeyboardInterrupt:
    # Clean up on Ctrl+C
    s.disable()
    print("Program stopped.")
