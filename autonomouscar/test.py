from testP import Printer
from testW import Writer


writer = Writer()
printer = Printer()
writer.startThread()
printer.startThread()
