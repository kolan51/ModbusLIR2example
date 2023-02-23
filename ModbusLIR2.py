# basic program example for LIR-2 modbus communication with PC

#Imports
import serial
import minimalmodbus
import sys
import glob
import time

# Checks if modbus connection is active
def checkCom():

    try:
        statusCom = lir2.read_register(0, 0, 4)

        print("LIR-2 FW_version: " + str(statusCom)[0:2] + "." + str(statusCom)[2:])

        if statusCom != 0:
            print("MSG: Successfullly communicated with lir sensor")
            return 1
        else:
            print("ERR: Failed to communicate with lir sensor")
            return 0

    except serial.SerialException as e:
        print("ERR: Failed to communicate with lir sensor")
        return 0


# Setup Modbus init parameters
def initCom():

    lir2.serial.port = 'COM3'  # this is the serial port name, correct it to your appropriate one
    lir2.serial.baudrate = 115200  # Baud Rate is 115200 by default
    lir2.serial.bytesize = 8
    lir2.serial.parity = serial.PARITY_NONE  # Parity is none by default
    lir2.serial.stopbits = 1  # Stopbits is set to 1 by default
    lir2.serial.timeout = 0.2  # Timeout in seconds seconds
    lir2.serial.write_timeout = 0.2  # Write timeout in seconds
    lir2.mode = minimalmodbus.MODE_RTU  # rtu mode by default
    lir2.close_port_after_each_call = True


# Function to read lir input register parameters
def readInputRegisters(inputRegistersWrite):

    for j in inputRegistersWrite:
        try:
            inReg = lir2.read_register(j, 0, 4)
            # print(inReg)
        except IOError:
            print("ERR: Failed to read lir sensor (input register)")


# Function to read lir holding register parameters
def readHoldingRegisters(holdingRegisters):

    for j in holdingRegisters:
        try:
            holReg = lir2.read_register(j, 0, 4)
            # print(holReg)
        except IOError:
            print("ERR: Failed to read lir sensor (holding register)")


# Function to write lir holding register parameters
def writeHoldingRegisters(holdingRegisters):

    for j in holdingRegisters:
        try:
            holReg = lir2.write_register(j, 0, 3)
            # print(holReg)
        except IOError:
            print("ERR: Failed to write lir sensor (holding register)")

# Read temperature data from Lir
def readDataLir():

    for i in range(9, 200, 96):
        try:
            inCurrent = lir2.read_registers(i, 96, 4)
            inCurrent = [x / 100 for x in inCurrent]  # To convert to degrees Celsium
            lirIn.extend(inCurrent)
            inCurrent = []
        except IOError:
            pass
            # print("ERR: Failed to read lir sensor (holding register) data")


# List serial ports on system (Windows\Linux\Mac OS)
def serialPorts():

    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


if __name__ == '__main__':

    # Init Lir
    try:
        print('Available serial ports (COM): ' + str(serialPorts()))
        lir2 = minimalmodbus.Instrument('COM3', 234)  # port name, slave address (in decimal)
        print("MSG: Successfully connected to lir sensor")
    except serial.SerialException as e:
        print("ERR: Failed to connect to lir sensor: ", e)

    # Global variables
    lirIn = []
    inputRegistersRead = list(range(9, 200))
    holdingRegistersRead = [800, 801, 802]
    holdingRegistersWrite = [800, 801, 802]

    # Modbus parameters init
    initCom()

    # Check connection and read lir data
    if checkCom() == 1:
        # inputRegistersRead(inputRegistersRead)
        # holdingRegistersWrite(holdingRegistersWrite)
        # holdingRegistersRead(holdingRegistersRead)

        # Infinitive loop to read temperatures from LIR
        while True:
            readDataLir()
            print(lirIn)
            lirIn = []
            time.sleep(1)