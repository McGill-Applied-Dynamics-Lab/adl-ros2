import math
import csv
import random
from functools import partial
import copy

from virtual_environment import EffectiveParams


class Packet:
    def __init__(self, info=-1, delay=-1, timeStamp=0):
        self.info: EffectiveParams = info
        self.delay = delay
        self.timeStamp = timeStamp
        self.timeTaken = 0


class CommunicationLine:
    def __init__(self):
        self.PacketList = [Packet(None, -1, 0)]

        my_data = self.data_open_and_copy("Haply_5G.csv")
        self.delay_distribution = self.get_bootstrap_func(my_data)

    def addToLine(self, info: EffectiveParams, step):
        if info is not None:
            delay = self.delay_distribution()
            newPacket = Packet(info, delay, step)
            if delay - 1 > len(self.PacketList):
                # pad out to delay time and insert new packet
                while len(self.PacketList) < delay - 1:
                    self.PacketList.append(copy.deepcopy(self.PacketList[-1]))
                    self.PacketList[-1].delay = self.PacketList[-2].delay + 1
            elif delay - 1 < len(self.PacketList):
                # cut to delay time and add new packet
                while len(self.PacketList) > delay - 1:
                    self.PacketList.pop(-1)
            self.PacketList.append(copy.deepcopy(newPacket))
        else:
            self.PacketList.append(copy.deepcopy(self.PacketList[-1]))

    def getCurrent(self) -> Packet:
        packet = self.PacketList.pop(0)
        return packet

    def seeLast(self):
        packet = copy.deepcopy(self.PacketList[-1])
        return packet

    def printLine(self):
        for packet in self.PacketList:
            print(f"info: {packet.state}, delay: {packet.delay}")

    def comm_5Gbootstrap(self, data):
        selection = random.randint(0, len(data) - 1)
        i = 0
        try:
            i = data[selection]
        except:
            stuff = 1
        try:
            x = math.ceil(i)
        except:
            stuff2 = 1
        return math.ceil(i)

    def constant_delay_distribution(self, constant):
        return constant

    def get_constant_delay_distribution(self, constant):
        return partial(self.constant_delay_distribution, constant)

    def get_bootstrap_func(self, all_data):
        return partial(self.comm_5Gbootstrap, all_data)

    def data_open_and_copy(self, data_file):
        all_data = []
        with open(data_file, "r", encoding="utf-8-sig", newline="") as data:
            csv_reader = csv.reader(data, delimiter=",")
            for row in csv_reader:
                all_data.append(float(row[0]))
        return all_data
