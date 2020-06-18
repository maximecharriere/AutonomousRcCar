from evdev import InputDevice, categorize, ecodes
import time
dev = InputDevice('/dev/input/event0')

time.sleep(5)
try:
    for event in dev.read():
        print(event)
except BlockingIOError:
    pass

print("ici")