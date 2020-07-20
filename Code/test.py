class Printer:
    def __init__(self, name):
        self.name =  name
    
    def printName(self):
        print(self.name)



traffic_objects = dict.fromkeys([0, 'Battery'], Printer('Battery')) 
traffic_objects.update(dict.fromkeys([1, 'SpeedLimit25'], Printer('SpeedLimit25')))
traffic_objects.update(dict.fromkeys([2, 'SpeedLimit50'], Printer('SpeedLimit50')))
traffic_objects.update(dict.fromkeys([3, 'StopSign'], Printer('StopSign')))
traffic_objects.update(dict.fromkeys([4, 'TrafficLightGreen'], Printer('TrafficLightGreen')))

for x in (set(traffic_objects.values())):
    x.printName()

map(lambda x: x.printName(), set(traffic_objects.values())) 