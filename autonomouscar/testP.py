from threading import Thread
import time

class Printer:
    def __init__(self):
        self.init()
        
    def init(self):
        self.value = 50


printer = Printer()

print(printer.value)