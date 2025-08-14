import serial
import math

# Configuración de la comunicación serial
port = '/dev/ttyUSB0'
baudrate = 230400

try:
    # Inicializar la comunicación serial
    ser = serial.Serial(port, baudrate, timeout=1)

    if ser.is_open:
        print(f'Conexión establecida en {port} a {baudrate} bps')

        # Escribir la letra "a"
        ser.write("b".encode())
        print('Se ha escrito "b" en el dispositivo')
        while True: 
            data = []
            lectura = ser.read()
            if(b'\xFA' == lectura):
                lecturaDos = ser.read()
                if(b'\xA0' == lecturaDos):
                    data.append(lectura)
                    data.append(lecturaDos)
                    x = ser.read(2518)
                    print(type(x))
                    data.append(x)
                    for i in range(0, 2520, 42):
                        if data[i] == 0xFA and data[i + 1] == 0xA0 + (i // 42):
                            for j in range(i + 4, i + 40, 6):
                                rangeA = data[j + 2]
                                rangeB = data[j + 3]
                                Degrees = 6 * (i // 42) + (j - 4 - i) // 6
                                range_val = (rangeB << 8) + rangeA
                                if Degrees != 0 and range_val != 0:
                                    Radians = (Degrees * math.pi) / 180
                                    x = range_val * math.cos(Radians)
                                    y = range_val * math.sin(Radians)
                                    print(f"{x},{y}")

    else:
        print('No se pudo abrir el puerto serial')

except serial.SerialException as e:
    print(f'Error al abrir el puerto serial: {e}')

    
finally:
    if ser.is_open:
        ser.close()
        print('Conexión cerrada')
