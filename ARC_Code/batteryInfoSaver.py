#!/usr/bin/env python3

from pijuice import PiJuice # Import pijuice module
import datetime

PiJuice = PiJuice() # Instantiate PiJuice interface object
Status = PiJuice.status.GetStatus()
ChargeLevel = PiJuice.status.GetChargeLevel()
BatteryVoltage = PiJuice.status.GetBatteryVoltage()
BatteryCurrent = PiJuice.status.GetBatteryCurrent()
IoVoltage = PiJuice.status.GetIoVoltage()
IoCurrent = PiJuice.status.GetIoCurrent()
BatteryTemperature = PiJuice.status.GetBatteryTemperature()

print(f"{datetime.datetime.now().strftime('%d.%m.%Y %H:%M:%S'):<22},\
    {Status['data']['battery']:<18},\
    {Status['data']['powerInput']:<13},\
    {Status['data']['powerInput5vIo']:<13},\
    {ChargeLevel['data']:<4},\
    {BatteryVoltage['data']:<6},\
    {BatteryCurrent['data']:<6},\
    {IoVoltage['data']:<6},\
    {IoCurrent['data']:<6},\
    {BatteryTemperature['data']}")