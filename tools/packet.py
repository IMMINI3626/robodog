from threading import RLock
from .deflib import (
    DefLib,
    JDCODE,
    JCBOARD,
    JDMISSION,
    UGLYBOT,
    JDCODE_CMD,
    ROBODOG,
)


class Packet:
    def __init__(self, model=JDCODE):
        self._lock = RLock()
        self.model = model
        self.packet = bytearray(20)
        if self.model == JDCODE:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0xB1, 0x14]
        elif self.model == JCBOARD:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0xC1, 0x14]
        elif self.model == JDMISSION:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0xD1, 0x14]
        elif self.model == UGLYBOT:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0xE1, 0x14]
        elif self.model == JDCODE_CMD:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0xB2, 0x14]
        elif self.model == ROBODOG:
            self.packet = bytearray(0x30)
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0x81, 0x30]
        else:
            self.packet[0:5] = [0x26, 0xA8, 0x14, 0x00, 0x14]

    def getCopiedPacket(self):
        with self._lock:
            return bytearray(self.packet)

    def getPacket(self):
        return self.packet

    def updatePacket(self, updater, recalculate_checksum=True):
        with self._lock:
            updater(self.packet)
            if recalculate_checksum:
                self.packet[5] = DefLib.checksum(self.packet)
            return bytearray(self.packet)

    def makePacket(self, start, data):
        if start + len(data) > len(self.packet):
            return None

        def updater(packet):
            for n in range(start, start + len(data)):
                packet[n] = data[n - start]

        return self.updatePacket(updater)

    def clearPacket(self):
        def updater(packet):
            end = len(packet)
            for n in range(5, end):
                packet[n] = 0
            if self.model == JDCODE:
                packet[14] = 0x01
                packet[16] = packet[18] = 0x64

        return self.updatePacket(updater)
