import sys
import termios
from ._sx126x import SX126x
from telemetry.msg import command_msg
from AX25UI import AX25UIFrameDecoder

class Transceiver(SX126x):
    def __init__(
            self, 
            serial_num="/dev/ttyS0", 
            freq=433, 
            addr=0, 
            power=22, 
            rssi=False, 
            air_speed=2400, 
            relay=False
        ) -> None:
        # Initialize the parent class (SX126x)
        super().__init__(serial_num=serial_num, freq=freq, addr=addr, power=power, rssi=rssi, air_speed=air_speed, relay=relay)
        
        # Save terminal settings to restore later
        self.old_settings = termios.tcgetattr(sys.stdin)

    def send_deal(
            self, 
            message: str  # CHANGE THIS
        ) -> None:
        """Sends a message after encoding

        Args:
            message (str): The message to be sent

        Returns:
            None
        """
        DEFAULT_ADDRESS = 0
        offset_frequency = self.freq - (850 if self.freq > 850 else 410)
        data = (bytes([DEFAULT_ADDRESS >> 8]) + 
                bytes([DEFAULT_ADDRESS & 0xff]) +
                bytes([offset_frequency]) + 
                bytes([self.addr >> 8]) + 
                bytes([self.addr & 0xff]) + 
                bytes([self.offset_freq]) + 
                message)
        # Send the encoded data
        self.send(data)
        return None

    def receive_data(self):
        # Receive data from the transceiver
        data = self.receive()

        # Process the received data if available
        if data:
            print(f"Data: {data}")
            try:
                # Decode the data to determine message type
                if isinstance(data, bytes):
                    decoder = AX25UIFrameDecoder()
                    decoded_frame = decoder.decode_ax25_frame(data)
                    message = decoded_frame["info"].split(",")

                # Check if the message is an uplink command
                if len(message) == 3:
                    component, component_id, command = message
                    uplink_command = command_msg()
                    uplink_command.component = component
                    uplink_command.component_id = int(component_id)
                    uplink_command.command = command
                    return uplink_command
                else:
                    return None
            except Exception as e:
                print(f"Error handling received data: {str(e)}")

    def reset_terminal_settings(self):
        """Reset terminal settings to their original state."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return None
