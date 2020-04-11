import asyncio
import myLib
from evdev import InputDevice, categorize, ecodes ,util
from PwmController import SteeringController
from PwmController import SpeedController

"""Pin declaration with BCM format"""
PIN_SPEED = 18
PIN_STEERING = 19

Controller = InputDevice('/dev/input/event3')




SpeedCtrl = SpeedController(PIN_SPEED,5.5,9.5)
SteeringCtrl = SteeringController(PIN_STEERING,5.5,9.5)

async def EventManager(device):
    async for event in device.async_read_loop():
        if event.type == ecodes.EV_ABS:
            if  event.code == ecodes.ABS_X:  #Joy Gauche / Gauche- Droite+
                print("HERE")
                SteeringCtrl.Angle(myLib.Map(event.value, 0, 2**16, 0, 100))
            elif  event.code == ecodes.ABS_Y: #Joy Gauche / Haut- Bas+
                SpeedCtrl.Speed(myLib.Map(event.value, 0, 2**16, 0, 100))
        #    elif event.code == ecodes.ABS_RX: #Joy Droit / Gauche- Droite+
        #        print("Joy Droit / Gauche- Droite+")
        #    elif  event.code == ecodes.ABS_RY: #Joy Droit / Haut- Bas+
        #        print("Joy Droit / Haut- Bas+")
        #    elif  event.code == ecodes.ABS_RZ: #Gachette droite
        #         print("Gachette droite")
        #    elif  event.code == ecodes.ABS_Z: #Gachette gauche
        #         print("Gachette gauche")
        #    elif  event.code == ecodes.ABS_HAT0X:
        #        if event.value == 1: #Croix / Droite
        #            print("Croix / Droite")
        #        elif  event.value == -1: #Croix / Gauche
        #            print("Croix / Gauche")
        #    elif  event.code == ecodes.ABS_HAT0Y:
        #        if event.value == 1: #Croix / Bas
        #            print("Croix / Bas")
        #        elif  event.value == -1: #Croix / Haut
        #            print("Croix / Haut")
        #elif event.type == ecodes.EV_KEY:
        #    if event.value == True:
        #        if event.code == ecodes.BTN_X:
        #            print("Bouton X")
        #        elif  event.code == ecodes.BTN_Y:
        #            print("Bouton Y")
        #        elif  event.code == ecodes.BTN_B:
        #            print("Bouton B")
        #        elif  event.code == ecodes.BTN_A:
        #            print("Bouton A")
        #        elif  event.code == ecodes.BTN_THUMBR:
        #            print("Bouton Joy R")
        #        elif  event.code == ecodes.BTN_THUMBL:
        #            print("Bouton Joy L")
        #        elif  event.code == ecodes.BTN_START:
        #            print("Bouton Menu")
        #        elif  event.code == ecodes.BTN_SELECT:
        #            print("Bouton Double Fenetre")
        #        elif  event.code == ecodes.BTN_MODE:
        #            print("Bouton Power")
        #        elif  event.code == ecodes.BTN_TR:
        #            print("Bouton gachette droite")
        #        elif  event.code == ecodes.BTN_TL:
        #            print("Bouton gachette gauche")

loop = asyncio.get_event_loop()
loop.run_until_complete(EventManager(Controller))
