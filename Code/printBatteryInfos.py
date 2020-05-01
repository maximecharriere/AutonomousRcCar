#!/usr/bin/env python3

## ----------------------------------- Infos -----------------------------------
#   Author:            Maxime Charriere
#   Project:           Autonomous RC Car
#   File:              printBatteryInfos.py
#   Link:              https://github.com/maximecharriere/AutonomousRcCar
#   Creation date :    18.04.2020
#   Last modif date:   01.05.2020
## ----------------------------------- Infos -----------------------------------

## -------------------------------- Description --------------------------------
#   This file prints the main information about the status of the battery
#   It is added in the crontab file to be executed every minute
#   and save this information in a csv file
## -------------------------------- Description --------------------------------

## ------------------------------ !! IMPORTANT !! ------------------------------
#   Don't rename or move this file. If you still want to do it, 
#   change the directory specified in this line of the crontab file.
#  
#   * * * * * /home/pi/Documents/AutonomousRcCar/ARC_Code/batteryInfoSaver.py 
#   >> /home/pi/Documents/AutonomousRcCar/BatteryInfo/BatteryInfoError.txt 
#   >> /home/pi/Documents/AutonomousRcCar/BatteryInfo/BatteryInfoSave.csv
#
#   Change contab with commande "crontab -e"
## ------------------------------ !! IMPORTANT !! ------------------------------


from pijuice import PiJuice # Import pijuice module
import datetime

# Get PiJuice infos
PiJuice             = PiJuice() # Instantiate PiJuice interface object
Status              = PiJuice.status.GetStatus()
ChargeLevel         = PiJuice.status.GetChargeLevel()
BatteryVoltage      = PiJuice.status.GetBatteryVoltage()
BatteryCurrent      = PiJuice.status.GetBatteryCurrent()
IoVoltage           = PiJuice.status.GetIoVoltage()
IoCurrent           = PiJuice.status.GetIoCurrent()
BatteryTemperature  = PiJuice.status.GetBatteryTemperature()

# Print infos
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