from typing import Optional

PROTOCOL_SERVO = 0x40
PROTOCOL_CHANNELS = 14
PROTOCOL_OVERHEAD = 3
PROTOCOL_LENGTH = 0x20


class IBus:

    def __init__(self, uart) -> None:
        self.uart = uart

    @staticmethod
    def _checksum(data: bytes, initial: int) -> tuple[int, int]:
        '''Compute IBUS checksum (two bytes).'''
        total = initial + sum(data)
        checksum = 0xFFFF - total
        return (checksum >> 8) & 0xFF, checksum & 0xFF

    def _read_byte(self) -> int:
        '''Blocking read of a single byte from UART.'''
        while True:
            data = self.uart.read(1)
            if data:
                return data[0]

    def _read_frame(self) -> Optional[list[int]]:
        '''Read a single IBUS frame from UART.'''

        # First byte is length
        frame_length = self._read_byte() - 1
        if not (PROTOCOL_OVERHEAD <= frame_length < PROTOCOL_LENGTH):
            return None

        # Read remaining frame
        buffer = bytearray(frame_length)
        bytes_read = self.uart.readinto(buffer)
        if bytes_read != frame_length:
            return None

        # Parse frame
        command = buffer[0] & 0xF0

        # Extract checksum (last two bytes)
        received_chA, received_chB = buffer[-1], buffer[-2]
        calc_chA, calc_chB = self._checksum(buffer[:-2], frame_length + 1)

        if (received_chA, received_chB) != (calc_chA, calc_chB):
            return None

        if command == PROTOCOL_SERVO:
            channels = [
                (buffer[2 * i + 2] << 8) | buffer[2 * i + 1]
                for i in range(PROTOCOL_CHANNELS)
            ]
            return channels

        return None
