#!/usr/bin/env python3
import os

pwm_BCM2number = {
    18:0,
    19:1
}

class HardPWMException(Exception):
    pass

class HardPWM(object):

    chippath = "/sys/class/pwm/pwmchip0"
    def __init__(self,pwm_BCMpin):
        self.pwm_num = pwm_BCM2number[pwm_BCMpin]
        self.pwmdir=f"{self.chippath}/pwm{self.pwm_num}"
        if not self.overlay_loaded():
            raise HardPWMException("Need to add 'dtoverlay=pwm-2chan' to /boot/config.txt and reboot") 
        self.create_pwmX()
        return

    def __del__(self):
        self.disable()
        self.delete_pwmX()

    def overlay_loaded(self):
        return os.path.isdir(self.chippath)   

    def sudo_echo(self,text,file, output=True):
        command_line = f"sudo sh -c 'echo {text} > {file}'"
        if not output:
            command_line += " > /dev/null 2>&1"
        return os.system(command_line)

    def create_pwmX(self):
        pwmexport = f"{self.chippath}/export"
        self.sudo_echo(self.pwm_num,pwmexport, output=False)

    def delete_pwmX(self):
        pwmunexport = f"{self.chippath}/unexport"
        self.sudo_echo(self.pwm_num,pwmunexport)

    def enable(self):
        enable = f"{self.pwmdir}/enable"
        self.sudo_echo(1,enable)    

    def disable(self):
        enable = f"{self.pwmdir}/enable"
        self.sudo_echo(0,enable)    

    def set_duty_cycle(self,milliseconds):
        dc = int(milliseconds * 1000000) #in ns
        duty_cycle = f"{self.pwmdir}/duty_cycle"
        self.sudo_echo(dc,duty_cycle)    

    def set_frequency(self,hz):
        per = int((1 / float(hz)) * 1000000000)
        period = f"{self.pwmdir}/period"
        self.sudo_echo(per,period)
        

if __name__ == "__main__":
    # Test code
    from time import sleep
    import numpy as np
    FREQ=50 
    S=1
    E=2 
    pwm = HardPWM(19)
    pwm.set_frequency(FREQ)
    pwm.set_duty_cycle(S)
    pwm.enable()  
    for dc in np.linspace(S, E, num=100, endpoint=True):
        pwm.set_duty_cycle(dc)
        sleep(0.1)