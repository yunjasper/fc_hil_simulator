"""
altos_flight_data.py

imports data from csv file exported from altos containing flight data
to be used in the simulation for the boost / coast phases.

author: jasper yun
"""

import numpy as np
import os
import pandas as pd

# for testing
data_filename = "flight_data/2018-04-08-serial-1257-flight-0014.csv"
data_filename = "flight_data/2018-04-22-serial-4225-flight-0015.csv"
data_filename = "flight_data/2018-08-11-serial-4263-flight-0001.csv"
data_filename = "flight_data/2017-06-03-serial-3418-flight-0001.csv"
data_filename = "flight_data/2019-03-31-serial-3418-flight-0003.csv"
data_filename = "flight_data/2019-05-04-serial-1257-flight-0017.csv"
data_filename = "flight_data/2019-05-26-serial-1257-flight-0018.csv"
data_filename = "flight_data/2019-09-01-serial-3418-flight-0004.csv"
data_filename = "flight_data/2019-11-09-serial-3462-flight-0005.csv"
data_filename = "flight_data/2019-11-10-serial-4357-flight-0003.csv"
data_filename = "flight_data/2020-07-18-serial-5018-flight-0011.csv"
data_filename = "flight_data/2020-08-01-serial-5129-flight-0003.csv"
data_filename = "flight_data/2020-08-08-serial-4411-flight-0002.csv"
data_filename = "flight_data/2020-09-05-serial-4967-flight-0002.csv"
data_filename = "flight_data/2020-10-10-serial-2378-flight-0021.csv" # FAILS!! EARLY EJECTION

# possible useful column names. altos flight data may not have every column.
# altitude appears twice, second is GPS altitude. units unclear. probably imperial.
possible_column_names = [
    'time',
    'state',
    'state_name',
    'acceleration',
    'pressure',
    'altitude',
    'height',
    'speed',
    'temperature',
    'drogue_voltage',
    'main_voltage',
    'battery_voltage',
    'accel_x',
    'accel_y',
    'accel_z',
    'gyro_roll',
    'gyro_pitch',
    'gyro_yaw',
    'mag_x',
    'mag_y',
    'mag_z',
    'tilt',
    'pyro',
    'latitude',
    'longitude',
    'altitude', # gps altitude, might have problems with duplicates?
]

def import_flight_data(filename, data_column_names) -> pd.DataFrame:
    # verify column names are ok
    for name in data_column_names:
        if not (name in possible_column_names):
            print(f'Error parsing flight data: desired column ({str(name)}) is not in the list of possible column names')
    
    # altitude is almost guaranteed to be in the file. use altitude to find apogee
    if 'altitude' not in data_column_names:
        data_column_names.append('altitude')

    try:
        df = pd.read_csv(filename)
    except:
        print('Could not read AltOs flight data -- bad filename')
        return None

    # extract the desired columns
    data = pd.DataFrame() # empty
    for col in data_column_names:
        try:
            data[col] = df[col]
        except:
            print(f'Error: Desired column ({str(col)}) is not in the file.')

    return data

def extract_boost_coast_phase_data(data : pd.DataFrame) -> int:
    altitude = data['altitude']
    # find index of apogee -- TODO: may require more robust searching
    apogee_index = altitude.argmax(axis=0)
    return apogee_index

def get_flight_data(filename, data_column_names):
    """
    returns dataframe containing desired data and index of apogee
    or None if error occurs (bad filename, desired data not present, etc.)
    """
    data = import_flight_data(filename, data_column_names)
    if data.empty:
        apogee_index = 0
    else:
        apogee_index = extract_boost_coast_phase_data(data)
    return data, apogee_index

def main():
    # change folder to the folder where this file is saved
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    cols = ['time', 'accel_x', 'accel_y', 'accel_z', 'pressure']
    data, apogee_index = get_flight_data(data_filename, cols)
    a = 1

if __name__ == '__main__':
    main()
