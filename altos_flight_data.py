"""
altos_flight_data.py

imports data from csv file exported from altos containing flight data
to be used in the simulation for the boost / coast phases.

author: jasper yun
"""

import numpy as np
import os
import pandas as pd

class AltOS_Flight_Data:

    # for testing -- uncomment the one you want to use
    # data_filename = "flight_data/2018-04-08-serial-1257-flight-0014.csv"
    # data_filename = "flight_data/2018-04-22-serial-4225-flight-0015.csv"
    # data_filename = "flight_data/2018-08-11-serial-4263-flight-0001.csv"
    # data_filename = "flight_data/2017-06-03-serial-3418-flight-0001.csv"
    # data_filename = "flight_data/2019-03-31-serial-3418-flight-0003.csv"
    # data_filename = "flight_data/2019-05-04-serial-1257-flight-0017.csv"
    # data_filename = "flight_data/2019-05-26-serial-1257-flight-0018.csv"
    # data_filename = "flight_data/2019-09-01-serial-3418-flight-0004.csv"
    # data_filename = "flight_data/2019-11-09-serial-3462-flight-0005.csv"
    # data_filename = "flight_data/2019-11-10-serial-4357-flight-0003.csv"
    # data_filename = "flight_data/2020-07-18-serial-5018-flight-0011.csv"
    # data_filename = "flight_data/2020-08-01-serial-5129-flight-0003.csv"
    # data_filename = "flight_data/2020-08-08-serial-4411-flight-0002.csv"
    # data_filename = "flight_data/2020-09-05-serial-4967-flight-0002.csv"
    data_filename = "flight_data/2020-10-10-serial-2378-flight-0021.csv" # supersonic with mach dip

    # column names of flight data exported from Altus Metrum. flight data may not have every column.
    # units unclear, may be metric
    DATA_TYPES = {
        # key : value = name we want to use, actual column name in file
        'time' : 'time',
        'state_value' : 'state',
        'state_name' : 'state_name',
        'filtered_accel' : 'acceleration',
        'pressure' : 'pressure',
        'altitude' : 'altitude',
        'height' : 'height',
        'speed' : 'speed',
        'temperature' : 'temperature',
        'drogue_voltage' : 'drogue_voltage',
        'main_voltage' : 'main_voltage',
        'battery_voltage' : 'battery_voltage',
        'accel_x' : 'accel_x',
        'accel_y' : 'accel_y',
        'accel_z' : 'accel_z',
        'gyro_roll' : 'gyro_roll',
        'gyro_pitch' : 'gyro_pitch',
        'gyro_yaw' : 'gyro_yaw',
        'mag_x' : 'mag_x',
        'mag_y' : 'mag_y',
        'mag_z' : 'mag_z',
        'tilt' : 'tilt',
        'pyro_voltage' : 'pyro',
        'latitude' : 'latitude',
        'longitude' : 'longitude',
        'gps_altitude' : 'altitude' # gps altitude, might have problems with duplicates
    }

    def import_flight_data(self, filename, requested_column_names) -> pd.DataFrame:
        # verify data names are ok while grabbing the column names from the dictionary
        file_column_names = []
        for name in requested_column_names:
            if not (name in self.DATA_TYPES):
                print(f'Error parsing flight data: desired column ({str(name)}) is not in the list of possible column names')
            else:
                file_column_names.append(self.DATA_TYPES.get(name))
        
        # altitude is almost guaranteed to be in the file. use altitude to find apogee
        if 'altitude' not in requested_column_names:
            requested_column_names.append('altitude')
            file_column_names.append(self.DATA_TYPES.get('altitude'))

        try:
            df = pd.read_csv(filename)
        except:
            print('Error occured: could not read AltOs flight data using read_csv')
            return None

        # extract the desired columns
        data = pd.DataFrame() # empty
        for col in file_column_names:
            try:
                data[col] = df[col]
            except:
                print(f'Error: Desired column ({str(col)}) is not in the file.')
        data.columns = requested_column_names
        return data

    def extract_boost_coast_phase_data(self, data : pd.DataFrame) -> int:
        altitude = data['altitude']
        # find index of apogee -- TODO: may require more robust searching
        apogee_index = altitude.argmax(axis=0)
        return apogee_index

    def get_flight_data(self, filename, data_column_names):
        """
        returns dataframe containing desired data and index of apogee
        or empty dataframe if error occurs (bad filename, desired data not present, etc.)
        """
        data = self.import_flight_data(filename, data_column_names)
        if data.empty:
            apogee_index = 0
        else:
            apogee_index = self.extract_boost_coast_phase_data(data)
        return data, apogee_index

def main():
    # change folder to the folder where this file is saved
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    afd = AltOS_Flight_Data()
    cols = ['time', 'accel_x', 'accel_y', 'accel_z', 'pressure']
    data, apogee_index = afd.get_flight_data(afd.data_filename, cols)
    a = 1

if __name__ == '__main__':
    main()
