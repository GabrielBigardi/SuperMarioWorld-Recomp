"""LoROM image loader for the SMW recompiler."""
import hashlib

SMW_US_SHA1 = '6b47bb75d16514b6a476aa0c73a683a2a4c18765'


class Rom:
    def __init__(self, path):
        data = open(path, 'rb').read()
        if len(data) % 1024 == 512:
            data = data[512:]  # copier header
        self.data = data
        self.size = len(data)
        self.sha1 = hashlib.sha1(data).hexdigest()

    def offset(self, addr):
        """24-bit CPU address -> ROM file offset, or None if not ROM (LoROM)."""
        bank, off = (addr >> 16) & 0xFF, addr & 0xFFFF
        if bank in (0x7E, 0x7F):
            return None
        if off < 0x8000:
            if (bank & 0x7F) < 0x40:
                return None  # WRAM / IO / open bus
            if (bank & 0x7F) >= 0x70:
                return None  # SRAM
            return None
        return (((bank & 0x7F) << 15) | (off & 0x7FFF)) % self.size

    def read8(self, addr):
        o = self.offset(addr)
        return None if o is None else self.data[o]

    def read16(self, addr):
        lo, hi = self.read8(addr), self.read8((addr & 0xFF0000) | ((addr + 1) & 0xFFFF))
        return None if lo is None or hi is None else lo | (hi << 8)

    def read24(self, addr):
        lo = self.read16(addr)
        b = self.read8((addr & 0xFF0000) | ((addr + 2) & 0xFFFF))
        return None if lo is None or b is None else lo | (b << 16)

    @staticmethod
    def canon(addr):
        """Canonical code address: fold FastROM mirror banks $80-$FF to $00-$7F."""
        return addr & 0x7FFFFF if (addr >> 16) >= 0x80 and (addr >> 16) not in (0xFE, 0xFF) else addr
