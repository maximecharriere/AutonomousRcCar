#!/usr/bin/env python3
import os
import RPi.GPIO as GPIO

pwm_BCM2number = {
    18:0,
    19:1
}

def dc2pc(freq, dutycycle):
    '''return the equivalent in percent of the dutycycle'''
    period = 1/freq*1000 #in ms
    return 100/period*dutycycle


class _PwmInterface():
    def __init__(self,BCMpin, freq):
        self.BCMpin = BCMpin
        self.freq = freq #in Hz
        self.duty_cycle = 0 #in ms

    def __del__(self):
        self.disable()

    def enable(self):
        pass

    def disable(self):
        pass

    def set_duty_cycle(self,milliseconds):
        pass

    def set_frequency(self,hz):
        pass

class SoftPWM(_PwmInterface):
    def __init__(self,BCMpin, freq):
        _PwmInterface.__init__(self,BCMpin,freq)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.BCMpin,GPIO.OUT)
        self.pwm_ctrl = GPIO.PWM(BCMpin , self.freq)

    def __del__(self):
        _PwmInterface.__del__(self)

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


class HardPWM(_PwmInterface):

    chippath = "/sys/class/pwm/pwmchip0"
    def __init__(self,BCMpin, freq):
        _PwmInterface.__init__(self,BCMpin,freq)
        try:
            self.pwm_num = pwm_BCM2number[self.BCMpin]
        except KeyError:
            raise KeyError(f"Hardware PWM are valid on pins {pwm_BCM2number}")
        self.pwmdir=f"{self.chippath}/pwm{self.pwm_num}"
        if not self._overlay_loaded():
            raise HardPWMException("Need to add 'dtoverlay=pwm-2chan' to /boot/config.txt and reboot") 
        self._create_pwmX()
        self.set_frequency(self.freq)
        return

    def __del__(self):
        _PwmInterface.__del__(self)
        self._delete_pwmX()

    def enable(self):
        enable_file = f"{self.pwmdir}/enable"
        self._sudo_echo(1,enable_file)

    def disable(self):
        enable_file = f"{self.pwmdir}/enable"
        self._sudo_echo(0,enable_file)    

    def set_duty_cycle(self,milliseconds):
        self.duty_cycle = milliseconds
        dutycycle_ns = int(self.duty_cycle * 1000000) #in ns
        print(dutycycle_ns)
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
        command_line = f"sudo sh -c 'echo {text} > {file}'"
        if not output:
            command_line += " > /dev/null 2>&1"
        return os.system(command_line)

    def _create_pwmX(self):
        export_file = f"{self.chippath}/export"
        self._sudo_echo(self.pwm_num,export_file, output=False)

    def _delete_pwmX(self):
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
    pwm = HardPWM(PIN, FREQ)
    pwm.set_duty_cycle(start)
    pwm.enable()  
    for dc in np.linspace(start, end, num=100, endpoint=True):
        pwm.set_duty_cycle(dc)
        sleep(0.05)
    for dc in np.linspace(end, start, num=100, endpoint=True):
        pwm.set_duty_cycle(dc)
        sleep(0.05)