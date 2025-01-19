import time
from plasma import WS2812
from servo import servo2040
from pimoroni import Servo

# Create and start the LED bar
led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

# Initialize Servo 1
servo1 = Servo(servo2040.SERVO_1)

def sweep_servo():
    """Sweep servo from 0 to 180 degrees and back"""
    while True:
        # Sweep from 0 to 180
        for angle in range(0, 181, 2):
            servo1.value(angle/180)  # Convert angle to value between 0 and 1
            time.sleep(0.02)
        
        # Sweep from 180 to 0
        for angle in range(180, -1, -2):
            servo1.value(angle/180)  # Convert angle to value between 0 and 1
            time.sleep(0.02)

# Main loop
try:
    # Enable the servo
    servo1.enable()
    
    # Start the sweep
    sweep_servo()
    
except KeyboardInterrupt:
    # Clean up on Ctrl+C
    servo1.disable()
    print("Program stopped.")
