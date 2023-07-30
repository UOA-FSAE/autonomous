import serial


CR = '\015'


class CANFrame:
    def __init__(self, frame_id, length, data):
        self.frame_id = frame_id
        self.length = length
        self.data = data

    def __str__(self):
        return f'id: {self.frame_id} | length: {self.length} | data: {self.data}'

    def to_bytes(self):
        identifier = str(self.frame_id)

        if len(identifier) == 2:
            identifier = '0' + identifier
        elif len(identifier) == 1:
            identifier = '00' + identifier
        elif len(identifier) == 0:
            identifier = '000'

        length = str(self.length)
        data = b''
        for item in self.data:
            data += bytes(str(item), 'ascii')

        return bytes(identifier, 'ascii') + bytes(length, 'ascii') + data


class CANDapter:
    def __init__(self, port, debug=True):
        self.debug = debug
        self.port = port
        self.serial = serial.Serial(port)

    def send_can_message(self, can_frame: CANFrame):
        command = b'T' + can_frame.to_bytes()
        self.send_command(command)

    def send_command(self, command: bytes):
        if not self.serial.is_open:
            return False

        msg = b'\015' + command + b'\015'
        self.serial.write(msg)
        return_message = self.serial.read()

        if self.debug:
            print(f'------- CANDapter Command sent --------')
            print(f'Command sent: {msg}')
            print(f'Received Value: {return_message}')
            print(f'---------------------------------------')

        return return_message == b'\x06'

    def set_bitrate(self, rate: int) -> bool:
        """https://www.ewertenergy.com/products/candapter/downloads/candapter_manual.pdf
        set rate to one of the following:
            10 Kbit
            20 Kbit
            50 Kbit
            100 Kbit
            125 Kbit
            250 Kbit
            500 Kbit
            800 Kbit
            1 Mbit

        returns bool depending on if successfully set
        """
        if rate == 10:
            command = b'S0'
        elif rate == 20:
            command = b'S1'
        elif rate == 50:
            command = b'S2'
        elif rate == 100:
            command = b'S3'
        elif rate == 125:
            command = b'S4'
        elif rate == 250:
            command = b'S5'
        elif rate == 500:
            command = b'S6'
        elif rate == 800:
            command = b'S7'
        elif rate == 1000:
            command = b'S8'
        else:
            raise ValueError("Rate given is not available on this device")

        return self.send_command(command)

    def open_channel(self):
        return self.send_command(b'O')

    def close_channel(self):
        return self.send_command(b'C')

    def _read_until(self):
        return self.serial.read_until(expected=b'\r')

    def read_can_message(self):
        can_message = str(self._read_until())[3:-3]

        data = [''] * 8
        for idx, char in enumerate(can_message[4:20]):
            idx //= 2
            data[idx] += str(char)

        return CANFrame(
            can_message[0:3],
            can_message[3],
            data
        )
