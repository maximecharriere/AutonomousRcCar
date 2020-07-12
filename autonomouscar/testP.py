from threading import Thread
import time

class Printer:
    def startThread(self, writer):
        t = Thread(target=self._printVar, name="printer", args=(writer,))
        t.start()

    def _printVar(self, writer):
        while True:
            print(f"Thread var: {writer.var}")
            time.sleep(1)