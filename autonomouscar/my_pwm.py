#!/usr/bin/env python3
import os
import RPi.GPIO as GPIO

# Hard PWM use pwm number (0 or 1) and not the PIN number (typically 18 and 19)
# This dic. map both value
pwm_BCM2number = {
    18:0,
    19:1
}

def dc2pc(freq, dutycycle):
    '''return the equivalent in percent of the dutycycle'''
    period = 1/freq*1000 #in ms
    return 100/period*dutycycle


class _IPwm():
    def __init__(self,BCMpin, freq):
        self.BCMpin = BCMpin
        self.freq = freq #in Hz
        self.duty_cycle = 0 #in ms

    def __del__(self):
        self.disable()

    def enable(self):
        '''Activate the PWM output'''

    def disable(self):
        '''Disable the PWM output'''

    def set_duty_cycle(self,milliseconds):
        '''Set the dutycycle of the PWM
        He must be smaller than the PWM period (1/freq)'''

    def set_frequency(self,hz):
        '''Set the frequency of the PWM'''

class SoftPwm(_IPwm):
    def __init__(self,BCMpin, freq):
        _IPwm.__init__(self,BCMpin,freq)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BCMpin,GPIO.OUT)
        self.pwm_ctrl = GPIO.PWM(BCMpin , self.freq)

    def __del__(self):
        _IPwm.__del__(self)

    def enable(self):
        self.pwm_ctrl.start(dc2pc(self.freq, self.duty_cycle))

    def disable(self):
        self.pwm_ctrl.stop()

    def set_duty_cycle(self,milliseconds):
        self.duty_cycle = milliseconds
        self.pwm_ctrl.ChangeDutyCycle(dc2pc(self.freq, self.duty_cycle))

    def set_frequency(self,hz):
        self.freq = hz
        self.pwm_ctrl.ChangeFrequency(self.freq)


class HardPwm(_IPwm):

    chippath = "/sys/class/pwm/pwmchip0"
    def __init__(self,BCMpin, freq):
        _IPwm.__init__(self,BCMpin,freq)
        #Test if the PIN given is compatible with hard PWM (see RPi pinout)
        try:
            self.pwm_num = pwm_BCM2number[self.BCMpin]
        except KeyError:
            raise KeyError(f"Hardware PWM are only valid on pins {[key for key in pwm_BCM2number.keys()]}")
        self.pwmdir=f"{self.chippath}/pwm{self.pwm_num}"

        #Test if the overlay to communicate with the kernel is loaded
        if not self._overlay_loaded():
            raise HardPWMException("Need to add 'dtoverlay=pwm-2chan' to /boot/config.txt and reboot to enable PWM") 
        self._create_pwmX()
        self.set_frequency(self.freq)
        return

    def __del__(self):
        _IPwm.__del__(self)
        self._delete_pwmX()

    def enable(self):
        '''Activate the PWM output'''
        enable_file = f"{self.pwmdir}/enable"
        self._sudo_echo(1,enable_file)

    def disable(self):
        enable_file = f"{self.pwmdir}/enable"
        self._sudo_echo(0,enable_file)    

    def set_duty_cycle(self,milliseconds):
        self.duty_cycle = milliseconds
        dutycycle_ns = int(self.duty_cycle * 1000000) #in ns
        dutycycle_file = f"{self.pwmdir}/duty_cycle"
        self._sudo_echo(dutycycle_ns,dutycycle_file)    

    def set_frequency(self,hz):
        self.freq = hz
        period_ns = int(1 / self.freq * 1000000000)
        period_file = f"{self.pwmdir}/period"
        self._sudo_echo(period_ns,period_file)


    def _overlay_loaded(self):
        return os.path.isdir(self.chippath)   

    def _sudo_echo(self,text,file, output=True):
        '''Write "text" in "file" with root privileges'''
        command_line = f"sudo sh -c 'echo {text} > {file}'"
        if not output:
            command_line += " > /dev/null 2>&1"
        return os.system(command_line)

    def _create_pwmX(self):
        '''Creat the folder to remotely communicate with kernel'''
        export_file = f"{self.chippath}/export"
        self._sudo_echo(self.pwm_num,export_file, output=False)

    def _delete_pwmX(self):
        '''Delete the folder to remotely communicate with kernel'''
        unexport_file = f"{self.chippath}/unexport"
        self._sudo_echo(self.pwm_num,unexport_file)


class HardPWMException(Exception):
    pass
  

if __name__ == "__main__":
    # Test code
    from time import sleep
    import numpy as np
    FREQ=50 
    PIN = 11
    start=1.2
    end=1.8
    pwm = HardPwm(PIN, FREQ)
    pwm.enable()
    pwm.set_duty_cycle(start)
    pwm.enable()  
    for dc in np.linspace(start, end, num=100, endpoint=True):
        pwm.set_duty_cycle(dc)
        sleep(0.05)
    for dc in np.linspace(end, start, num=100, endpoint=True):
        pwm.set_duty_cycle(dc)
        sleep(0.05)