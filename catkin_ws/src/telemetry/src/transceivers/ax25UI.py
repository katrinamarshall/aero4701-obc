import struct
import binascii

class AX25UIFrame:
    FLAG = 0x7E
    CONTROL = 0x3F
    PID = 0xF0
    FCS_POLY = 0x8408

    def __init__(self, info, ssid_type):
        self.source = "DEBRA  "  # Padded to 6 characters
        self.destination = "GROUND "  # Padded to 6 characters
        self.ssid_type = ssid_type # Im thinking 0b1110 for WOD, 0b1111 for science, 0b1101 for satellite pose 0b1011 for payload data and 0b0111 for misc
        self.info = info

    def encode_address(self, address, ssid):
        address = address.ljust(6)
        encoded = bytearray()
        for char in address:
            encoded.append(ord(char) << 1)
        ssid_byte = 0b01100000 | (ssid << 1) | 1  # Set the last bit to 1
        encoded.append(ssid_byte)
        return encoded

    def create_frame(self):
        frame = bytearray()
        
        # Beginning flag
        frame.append(self.FLAG)

        # Destination address
        frame.extend(self.encode_address(self.destination, self.ssid_type))

        # Source address
        frame.extend(self.encode_address(self.source, 0b0000))

        # Control
        frame.append(self.CONTROL)

        # PID
        frame.append(self.PID)

        # Information
        frame.extend(self.info.encode('ascii'))

        # Compute and add FCS
        fcs = self.compute_fcs(frame[1:])
        frame.extend(fcs)

        # Ending flag
        frame.append(self.FLAG)

        return frame

    def compute_fcs(self, frame):
        fcs = 0xFFFF
        for byte in frame:
            fcs ^= byte
            for _ in range(8):
                if fcs & 0x01:
                    fcs = (fcs >> 1) ^ self.FCS_POLY
                else:
                    fcs >>= 1
        fcs = ~fcs & 0xFFFF
        return struct.pack('<H', fcs)

    def to_hex(self, frame):
        return ' '.join(format(x, '02x') for x in frame)

# # Example usage
# info = "Test Data"  # Information field to be filled by the user
# ssid_type = 0b1111  # Science Data

# ax25_frame = AX25UIFrame(info, ssid_type)
# frame = ax25_frame.create_frame()
# print(ax25_frame.to_hex(frame))
