# RECORD PDR MEASUREMENTS OF RASPBERRY PICO ADVERTISEMENTS FROM ARDUINO NANO ESP32

import os
import serial
import pandas as pd
import time

# Global variables
TOTAL_MEAS = 1  # Maximum number of measurements in the session, to prevent an infinite loop
MAX_RECORD = 500  # Specifies the number of measurements to be recorded during a measurement input 'm'
SER_PORT = 'COM10'  # Serial port the device is connected to
SER_BAUD = 115200  # Serial port baud rate
FILENAME = os.path.join(os.getcwd(), 'PDR_FC112_Test3.txt')  # Output file

class SerialPort:
    def __init__(self, port, baud=115200):
        if not isinstance(port, str):
            raise TypeError('Port must be a string.')

        if not isinstance(baud, int):
            raise TypeError('Baud rate must be an integer.')

        self.port = port
        self.baud = baud

        # Initialize serial connection
        self.ser = serial.Serial(self.port, self.baud, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)
        self.ser.flushInput()
        self.ser.flushOutput()

    def read(self, clean_end=True):
        self.ser.flushInput()
        bytesToRead = self.ser.readline()
        decodedMsg = bytesToRead.decode('utf-8')

        if clean_end:
            decodedMsg = decodedMsg.rstrip()  # Strip extra chars at the end

        return decodedMsg

    def close(self):
        self.ser.close()

def record_data(ser: SerialPort, max_record: int = MAX_RECORD):
    # RSSI measurement array
    data_list = []

    for _ in range(max_record):
        try:
            data = ser.read().split(',')
            if len(data) >= 2:
                x = float(data[0])
                y = float(data[1])  # Use square brackets to access list elements
                data_list.append(x)
                data_list.append(y)
            else:
                # Handle the case where the data doesn't have the expected format
                print("[ERROR]: Unexpected data format. Skipping this measurement.")
        except Exception as e:
            ser.close()
            raise SystemExit(f"[ERROR]: Error reading serial connection. Exception: {e}")

    return data_list

def list_to_delim_file(mylist: list, filename: str, delimiter: str = ',', f_mode='a'):
    # Convert list to Pandas dataframe, then save as a text file.
    df = pd.DataFrame(mylist)
    df.to_csv(filename, sep=delimiter, mode=f_mode, header=False, index=False)

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
            RSSI_list = record_data(ser)
            data.extend(RSSI_list)
            print('[INFO]: RSSI (dBm) = {}'.format(RSSI_list))
        elif user == 's':
            # Save data onto .txt file then quit
            print('[INFO]: Saving data and exiting...')
            list_to_delim_file(data, FILENAME, delimiter='\t')
            ser.close()
            elapsed_time = time.time() - start_time
            sampling_interval = elapsed_time / (TOTAL_MEAS*MAX_RECORD) # Divide total time with the total number of measurements taken
            print(f'[INFO]: Total time elapsed: {elapsed_time:.4f} seconds')
            print(f'[INFO]: Sampling interval: {sampling_interval:.4f} seconds per measurement')
            print('[INFO]: Done!')
            return
        elif user == 'exit':
            print('[WARNING]: Exiting program without saving!')
            ser.close()
            raise SystemExit("[INFO]: System closed!")
        else:
            print('[ERROR]: \'{}\' is an unknown input. Skipping this measurement.'.format(user))
            continue

    # Save once max is reached
    print('[WARNING]: Reached maximum number of datapoints, saving file...')
    elapsed_time = time.time() - start_time # Calculate how much time has passed
    sampling_interval = elapsed_time / (TOTAL_MEAS*MAX_RECORD) # Divide total time with the total number of measurements taken
    list_to_delim_file(data, FILENAME, delimiter='\t')
    ser.close()
    print(f'[INFO]: Total time elapsed: {elapsed_time:.4f} seconds')
    print(f'[INFO]: Sampling interval: {sampling_interval:.4f} seconds per measurement')
    print('[INFO]: Done!')

if __name__ == '__main__':
    main()
