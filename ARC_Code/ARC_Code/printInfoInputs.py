import asyncio
from evdev import InputDevice, categorize, ecodes ,util

Controller = InputDevice('/dev/input/event0')

async def EventManager(device):
    async for event in device.async_read_loop():
        if event.type == ecodes.EV_KEY:
            if event.value == True:
                 print(f"Touche: {ecodes.keys[event.code]}")
        elif event.type == ecodes.EV_ABS:
            print(f"Touche: {ecodes.ABS[event.code]}    Valeur: {event.value}")
         

loop = asyncio.get_event_loop()
loop.run_until_complete(EventManager(Controller))

