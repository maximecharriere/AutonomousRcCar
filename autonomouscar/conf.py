import yaml
from evdev import InputDevice, categorize, ecodes, util



with open('conf.yaml') as f:
    conf = yaml.load(f, Loader=yaml.FullLoader)


conf.PIN.speed