import struct

class AX25UIFrameDecoder:
    def decode_ax25_frame(self, frame):
        """Decode an AX.25 frame and extract the relevant fields"""
        # Ensure there are beginning and ending flags
        if frame[0] != 0x7E or frame[-1] != 0x7E:
            raise ValueError("Invalid AX.25 frame")

        # Address field - destination
        destination_callsign = ''.join([chr((frame[i] >> 1) & 0x7F) for i in range(1, 7)]).strip()
        destination_ssid = (frame[7] >> 1) & 0x0F

        # Address field - source
        source_callsign = ''.join([chr((frame[i] >> 1) & 0x7F) for i in range(8, 14)]).strip()
        source_ssid = (frame[14] >> 1) & 0x0F

        # Control
        control_field = frame[15]

        # PID
        pid_field = frame[16]

        # Info
        info_field = frame[17:-3].decode('ascii')

        # FCS
        received_fcs = frame[-3:-1]
        calculated_fcs = self.compute_fcs(frame[1:-3])

        if received_fcs != calculated_fcs:
            raise ValueError("FCS check failed")

        return {
            'd_call': destination_callsign,
            'd_ssid': destination_ssid,
            's_call': source_callsign,
            's_ssid': source_ssid,
            'control': control_field,
            'pid': pid_field,
            'info': info_field
        }

    def compute_fcs(self, frame):
        """Compute the Frame Check Sequence (FCS) for a given frame using the CRC-CCITT algorithm."""
        fcs = 0xFFFF
        for byte in frame:
            fcs ^= byte
            for _ in range(8):
                if fcs & 0x01:
                    fcs = (fcs >> 1) ^ 0x8408
                else:
                    fcs >>= 1
        fcs = ~fcs & 0xFFFF
        return struct.pack('<H', fcs)
