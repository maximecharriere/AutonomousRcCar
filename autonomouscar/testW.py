from threading import Thread
import numpy as np
import time
class Writer:
    var = 0
    def startThread(self):
        t = Thread(target=self._writeVar, name="writer", args=())
        t.start()

    def _writeVar(self):
        while True:
            self.var = input('var:')