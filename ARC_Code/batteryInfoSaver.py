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

print(f"{datetime.datetime.now().strftime('%d.%m.%Y %H:%M:%S')},\
    {Status['data']['battery']},\
    {Status['data']['powerInput']},\
    {Status['data']['powerInput5vIo']},\
    {ChargeLevel['data']},\
    {BatteryVoltage['data']},\
    {BatteryCurrent['data']},\
    {IoVoltage['data']},\
    {IoCurrent['data']},\
    {BatteryTemperature['data']}")