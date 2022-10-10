#!/usr/bin/python

# Importamos la libreira de PySerial
import serial

# Abrimos el puerto del arduino a 9600
PuertoSerie = serial.serial('/dev/ttyS0', 9600)
# Creamos un buble sin fin
while True:
  # leemos hasta que encontarmos el final de linea
  sArduino = PuertoSerie.readline()
  # Mostramos el valor leido y eliminamos el salto de linea del final
  print ("Valor Arduino: " + sArduino.rstrip('\n'))