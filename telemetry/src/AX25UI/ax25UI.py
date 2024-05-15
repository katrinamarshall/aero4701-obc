import struct
import binascii

""" 
The AX25UIFrame class creates a frame using the AX.25 protocol using a 
UI (Unnumbered Information) frame, based on an SSID which defines the 
data type of the message and info, which contains the message to be sent
"""
class AX25UIFrame:
    FLAG = 0x7E         # Flags are at then beginning and end of the frame
    CONTROL = 0x3F      # Control field is 3F for UI frames
    PID = 0xF0          # F0 for PID field specifies that no layer 3 protol is used
    FCS_POLY = 0x8408

    def __init__(self, info, ssid_type):
        self.source = "DEBRA"
        self.destination = "GROUND"
        self.ssid_type = ssid_type # Im thinking 0b1110 for WOD, 0b1111 for science, 0b1101 for satellite pose, 0b0111 for commands, 0b1011 for misc
        self.info = info

    def encode_address(self, callsign, ssid):
        """Each address (source and destination) within the address field
        must have a callsign that is 6 bytes long followed by an SSID that 
        is 1 byte long and specifies the data type of the message

        Args:
            callsign (str): String
            ssid (int): Data type of the message being sent

        Returns:
            _type_: _description_
        """
        # Address needs 6 bytes so must be padded
        callsign = callsign.ljust(6)
        encoded = bytearray()

        # Each character of the callsign must be bit shifted with a 0 at the end
        for char in callsign:
            encoded.append(ord(char) << 1)
        
        # The final bit of the address should be a 1 if it is the last callsign in the address field
        if(ssid == 0b0000):
            ssid_byte = 0b01100000 | (ssid << 1) | 1  
        else:
            ssid_byte = 0b01100000 | (ssid << 1)
        encoded.append(ssid_byte)
        return encoded

    def create_frame(self):
        """Creates the UI frame based on all the information specified"""
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
        frame.extend(self.info)#.encode('ascii'))

        # Compute and add FCS
        fcs = self.compute_fcs(frame[1:])
        frame.extend(fcs)

        # Ending flag
        frame.append(self.FLAG)

        return frame

    def compute_fcs(self, frame):
        """
        Compute the Frame Check Sequence (FCS) for a frame

        Args:
            frame (bytearray): The input frame for which the FCS is to be computed

        Returns:
            bytes: The computed FCS which is two bytes
        """
        # Initialize FCS to 0xFFFF
        fcs = 0xFFFF
        
        # Loop through each byte in the frame
        for byte in frame:
            # XOR the byte with the current FCS value
            fcs ^= byte
            
            # Perform bitwise operations for each of the 8 bits in the byte
            for _ in range(8):
                # Check if the least significant bit of the FCS is 1
                if fcs & 0x01:
                    # If it is 1, right shift the FCS and XOR with the polynomial
                    fcs = (fcs >> 1) ^ self.FCS_POLY
                else:
                    # If it is 0, simply right shift the FCS
                    fcs >>= 1
        
        # Invert all the bits in the FCS and mask to ensure it fits within 16 bits
        fcs = ~fcs & 0xFFFF
        
        # Return the FCS packed as a little-endian 16-bit unsigned integer
        return struct.pack('<H', fcs)


    def to_hex(self, frame):
        """Converts bytes to hexadecimal"""
        return ' '.join(format(x, '02x') for x in frame)