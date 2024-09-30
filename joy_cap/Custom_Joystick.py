import os
import struct
import fcntl

class JoystickReader:
    def __init__(self, js_id=0):
        self.__js_id = int(js_id)
        self.__jsdev = None
        self.__function_names = {
            'RK1_UP_DOWN': 0x0201,
            'RK2_LEFT_RIGHT': 0x0202,
            'X': 0x0103,
        }

        self.__current_values = {key: 0.0 for key in self.__function_names}
        self.__open_device()
        self.__make_non_blocking()

    def __open_device(self):
        try:
            js = f'/dev/input/js{self.__js_id}'
            self.__jsdev = open(js, 'rb')
            print(f'--- Opening {js} succeeded ---')
        except FileNotFoundError:
            print(f'--- Failed to open {js} ---')
            self.__jsdev = None

    def __make_non_blocking(self):
        """Configurar el dispositivo para lectura no bloqueante."""
        if self.__jsdev:
            fd = self.__jsdev.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    def __del__(self):
        if self.__jsdev:
            self.__jsdev.close()
            print("--- Joystick device closed ---")

    def update(self):
        """Leer el estado actual del joystick y actualizar los valores."""
        if not self.__jsdev:
            print("Joystick device not opened")
            return
        
        try:
            while True:
                evbuf = self.__jsdev.read(8)
                if not evbuf:
                    break
                timestamp, value, type, number = struct.unpack('IhBB', evbuf)
                func = type << 8 | number

                for name, code in self.__function_names.items():
                    if func == code:
                        if "UP_DOWN" in name or "LEFT_RIGHT" in name:
                            value = -value / 32767.0
                        self.__current_values[name] = value
        except BlockingIOError:
            # No hay nuevos eventos, continuar sin bloquear
            pass
        except Exception as e:
            print(f"Error al leer el joystick: {e}")
            self.reconnect()

    def get_value(self, param):
        return self.__current_values.get(param, None)

    def reconnect(self):
        if self.__jsdev:
            self.__jsdev.close()
        self.__open_device()
        self.__make_non_blocking()
