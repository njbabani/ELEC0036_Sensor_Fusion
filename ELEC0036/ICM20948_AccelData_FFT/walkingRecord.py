############################
#          BABANZ          #
############################
# RECORD LINEAR ACCELERATION MEASUREMENTS OF ACCELEROMETER FOR FFT THROUGH MATLAB

import os
import serial
import pandas as pd
import time

# Global variables
TOTAL_MEAS = 1  # Maximum number of measurements in the session, to prevent an infinite loop
MAX_RECORD = 2000 # Specifies the number of recordings during a measurement input 'm'
SER_PORT = 'COM15'  # Serial port the device is connected to 
SER_BAUD = 115200  # Serial port baud rate
FILENAME = os.path.join(os.getcwd(), 'walkingdata.txt')  # Output file

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

def recordData(ser: SerialPort, max_record: int = MAX_RECORD):
    # Initialize ax, ay, and az as 0.0 for float values
    linear_accel_list = []

    for _ in range(max_record):
        try:
            data = ser.Read().split(',')
            if len(data) >= 1:
                linear_accel = float(data[0])
                linear_accel_list.append(linear_accel)
            else:
                # Handle the case where the data doesn't have the expected format
                print("[ERROR]: Unexpected data format. Skipping this measurement.")
        except Exception as e:
            ser.Close()
            raise SystemExit(f"[ERROR]: Error reading serial connection. Exception: {e}")

    return linear_accel_list

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
    for _ in range(TOTAL_MEAS):
        user = input('[INPUT]: Type \'m\' to measure, \'s\' to save and quit, or \'exit\' to exit without saving: ').lower()
        if user == 'm':
            # Record data to list
            start_time = time.time()
            linear_accel_list = recordData(ser)
            data.extend(linear_accel_list)
            print('[INFO]: linearAccel (m/s^2) = {}'.format(linear_accel_list))
        elif user == 's':
            # Save data onto .txt file then quit
            print('[INFO]: Saving data and exiting...')
            List2DelimFile(data, FILENAME, delimiter='\t')
            ser.Close()
            elapsed_time = time.time() - start_time # Calculate how much time has passed
            sampling_interval = elapsed_time / (TOTAL_MEAS*MAX_RECORD) # Divide total time with the total number of measurements taken
            print(f'[INFO]: Total time elapsed: {elapsed_time:.4f} seconds')
            print(f'[INFO]: Sampling interval: {sampling_interval:.4f} seconds per measurement')
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
    print('[WARNING]: Reached the maximum number of data points, saving file...')
    elapsed_time = time.time() - start_time # Calculate how much time has passed
    sampling_interval = elapsed_time / (TOTAL_MEAS*MAX_RECORD) # Divide total time with the total number of measurements taken
    List2DelimFile(data, FILENAME, delimiter='\t')
    ser.Close()
    print(f'[INFO]: Total time elapsed: {elapsed_time:.4f} seconds')
    print(f'[INFO]: Sampling interval: {sampling_interval:.4f} seconds per measurement')
    print('[INFO]: Done!')

if __name__ == '__main__':
    main()