############################
#          BABANZ          #
############################
# RECORD ACCELEROMETER MEASUREMENTS FOR ACCELEROMETER CALIBRATION

import os
import math
import serial
import pandas as pd

# Global variables
MAX_AVG_MEAS = 1000  # Maximum number of average measurements in the session, to prevent an infinite loop
MAX_RECORD = 50 # Specifies the number of measurements to be recorded during a measurement input 'm'
AVG_MEAS = 1  # For each reading, take this many measurements and average them
SER_PORT = 'COM15'  # Serial port the device is connected to 
SER_BAUD = 115200  # Serial port baud rate
FILENAME = os.path.join(os.getcwd(), 'linearAcceldata.txt')  # Output file

class SerialPort:
    def __init__(self, port, baud=115200):
        if isinstance(port, str) == False:
            raise TypeError('Port must be a string.')

        if isinstance(baud, int) == False:
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        # Initialize serial connection
        self.ser = serial.Serial(self.port, self.baud, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
        self.ser.flushInput()
        self.ser.flushOutput()

    def Read(self, clean_end=True):
        self.ser.flushInput()
        bytesToRead = self.ser.readline()
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end == True:
            decodedMsg = decodedMsg.rstrip()  # Strip extra chars at the end

        return decodedMsg

    def Close(self) -> None:
        self.ser.close()

def recordData(ser: SerialPort):
    # Initialise ax, ay, and az as 0.0 for float values
    magRaw = magCal = 0.0
    for _ in range(AVG_MEAS):
        # Read data
        try:
            data = ser.Read().split(',')
            if len(data) >= 2:
                magRaw_now = float(data[0])
                magCal_now = float(data[1])
            else:
                # Handle the case where the data doesn't have the expected format
                print("[ERROR]: Unexpected data format. Skipping this measurement.")
                continue
        except Exception as e:
            ser.Close()
            raise SystemExit(f"[ERROR]: Error reading serial connection. Exception: {e}")
        magRaw += magRaw_now
        magCal += magCal_now

    return (magRaw / AVG_MEAS, magCal / AVG_MEAS)

def List2DelimFile(mylist: list, filename: str, delimiter: str = ',', f_mode = 'a'):
    # Convert list to Pandas dataframe, then save as a text file.
    df = pd.DataFrame(mylist)
    df.to_csv(filename, sep = delimiter, mode = f_mode, header = False, index = False)

def main():
    ser = SerialPort(SER_PORT, baud=SER_BAUD)
    data = []  # Data list

    print('[INFO]: Place sensor level and stationary on desk.')
    input('[INPUT]: Press any ENTER to continue...')

    # Obtain measurements
    for _ in range(MAX_AVG_MEAS):
        user = input('[INPUT]: Type \'m\' to measure, \'s\' to save and quit, or \'exit\' to exit without saving: ').lower()
        if user == 'm':
            # Record data to list
            for _ in range(MAX_RECORD):
                magRaw, magCal = recordData(ser)
                print('[INFO]: Avgd Readings: {:.4f}, {:.4f}'.format(magRaw, magCal))
                data.append([magRaw, magCal])
        elif user == 's': 
            # Save data onto .txt file then quit
            print('[INFO]: Saving data and exiting...')
            List2DelimFile(data, FILENAME, delimiter='\t')
            ser.Close()
            print('[INFO]: Done!')
            return
        elif user == 'exit':
            print('[WARNING]: Exiting program without saving!')
            ser.Close()
            raise SystemExit("[INFO]: System closed!")
        else:
            print('[ERROR]: \'{}\' is an unknown input. Skipping this measurement.'.format(user))
            continue

    # Save once max is reached
    print('[WARNING]: Reached maximum number of datapoints, saving file...')
    List2DelimFile(data, FILENAME, delimiter='\t')
    ser.Close()
    print('[INFO]: Done!')

if __name__ == '__main__':
    main()