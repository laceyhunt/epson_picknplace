"""
modbus_fxns

all modbus helper functions used in main.py
Note: all locs are off by 1 from the Epson side b/c Epson is not 0-indexed
"""
import numpy as np
import cv2 as cv
import math
import time
import platform
import os
from enum import Enum
import csv
from datetime import datetime
import pymodbus.client as ModbusClient
from pymodbus import (
    ExceptionResponse,
    FramerType,
    ModbusException,
    pymodbus_apply_logging_config,
)
log = False
robot_ip = '192.168.0.1'
# May not need:
CALIBRATION_TEST_MODE = False
INTEGRATION_TEST_MODE = True
CONV_TEST_MODE = False
SMALL_TEST_MODE = False
DEBUG_MODE = False
IMAGE_TEST_MODE = True
DISPLAY_MODE = False


# Modbus Constants
#--BITS
CYCLE_COMPLETE = 511
START_COMMAND = 512
CONVEYOR_ON = 513
# CONV_INCLINE = 514
# AGITATORS = 515
# BURST = 516

#--REGISTERS
ROBOT_CYCLE_COMPLETE = 31
class RobotState(Enum):
    OFF_STATE = "off_state"
    WAITING_STATE = "waiting_state"
    MOVING_STATE = "moving_state"
    ERROR_STATE = "error_state"
    NO_COMM_STATE = "no_comm_state"

#Datalogging, unused for now
if log is True:
    csv_file = 'item_location_data.csv'
    file_exists = os.path.isfile(csv_file)
    print(file_exists)

#Data logging functions, unused for now
def log_seal_data(total, rejected_width):  #, rejected_overlap, rejected_edge):
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    #Open the file in append mode
    with open(csv_file, mode= 'a', newline='') as file:
        writer = csv.writer(file)

        if not file_exists:
            writer.writerow(['total', 'rejected_width', 'time_date'])

        writer.writerow([total, rejected_width, current_time])

def initialize_modbus(comm):
    """Run sync client."""
    # activate debugging
    if DEBUG_MODE:
        pymodbus_apply_logging_config("DEBUG")
    framer=FramerType.SOCKET
    port = "502"

    print("get client")
    if comm == "tcp":
        client = ModbusClient.ModbusTcpClient(
            robot_ip,
            port=port,
            framer=framer,
            # timeout=10,
            # retries=3,
            # source_address=("localhost", 0),
        )

    print("connect to server")
    client.connect()
    return client

def set_modbus_bit(client, address, command):
    """
    set_modbus_bit
    
    :param client
    :param address: where to set
    :param command: what to set it as
    """
    print(f"Sending address {address} command {command}")
    try:
        rr = client.write_coil(address, command)#, slave=1)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        return

    if rr.isError():
        print(f"Received Modbus library error({rr})")
        return

    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        return

    print(f"Address {address} command {command} successfully sent.")

def reset_bits(client, max_items=33, max_items_bad=15):
    """
    reset_bits

    resets all values to 0, used at the start and end of every cycle
    
    :param client
    :param max_items: max good item values to reset (for resetting x and y to 0)
    :param max_items_bad: max bad item values to reset (for resetting x and y to 0)
    """
    set_modbus_bit(client, START_COMMAND, 0)
    set_modbus_bit(client, CONVEYOR_ON, 0)
    print("Resetting old coords...")
    send_target_count(client, max_items)
    for i in range(max_items):
        send_modbus_coords(client, i, 0, 0)
    send_target_count(client, max_items_bad,False)
    for i in range(max_items_bad):
        send_modbus_coords(client, i, 0, 0,False)

def conveyor(client, status):
    """
    conveyor
    
    controls conveyor_on bit for Epson I/O
    :param client
    :param status: 1=on, 0=off
    """
    if status == 'on':
        set_modbus_bit(client, CONVEYOR_ON, 1)
    elif status == 'off':
        set_modbus_bit(client, CONVEYOR_ON, 0)

def send_target_count(client, target_count, good=True):
    """
    send_target_count
    
    sends the num of items to palletize 
    :param client
    :param target_count: number to set
    :param good: True=set 'num_bottles' value; False=set 'num_bottles_bad' value
    """
    if target_count < 0 or target_count > 40:
        print("Invalid Target Count.")
        return
    # Modbus register address for the target count
    if good:
        mb_target_count_register = 99 # Target count register
    else:
        mb_target_count_register = 129 # Target count register for bad items
    print(f"Sending target count: {target_count}")
    try:
        rr = client.write_register(mb_target_count_register, target_count)#, slave=1)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        return
    if rr.isError():
        print(f"Received Modbus library error({rr})")
        return
    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        return
    print(f"Target count {target_count} successfully sent.")

def to_int16_and_scale(value):
    """
    to_int16_and_scale
        scales x/y value to fit in Modbus register (unscaled on the Epson side)
        need to be unsigned, two decimals of accuracy
    :param value: value to convert
    """
    value*=100
    value = int(value)
    if not (0<=value<=0xFFFF):
        print("error scaling!")
        return 0
    return value

def send_modbus_coords(client, target_num, x_coord, y_coord, good=True):
    """
    send_modbus_coords
    
    sends the world coordinates of either good (good=True) or bad (good=False) items to Epson locations
    
    :param client
    :param target_num: coordinate number
    :param x_coord: x coord
    :param y_coord: y coord
    :param good: True=white, False=orange
    """
    # print(f"sending coord {target_num}")
    if target_num == 0:
        return
    
    if good:
        start = 32
    else:
        start = 100
    # Modbus register addresses for x and y coordinates
    mb_x_register = start + 2 * (target_num - 1)  # X register for target_num
    mb_y_register = mb_x_register + 1          # Y register for target_num

    # Scale coordinates and convert to integers
    # Has to be unsigned 16bit...
    mb_x_coordinate = to_int16_and_scale(x_coord) #int(x_coord * 100)
    mb_y_coordinate = to_int16_and_scale(y_coord) #int(y_coord * 100)

    # print("send and verify x data")
    try:
        rr = client.write_register(mb_x_register, mb_x_coordinate)#, slave=1)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        return

    if rr.isError():
        print(f"Received Modbus library error({rr})")
        return

    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        return

    # print("send and verify y data")
    try:
        rr = client.write_register(mb_y_register, mb_y_coordinate)#, slave=1)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        return

    if rr.isError():
        print(f"Received Modbus library error({rr})")
        return

    if isinstance(rr, ExceptionResponse):
        print(f"Received Modbus library exception ({rr})")
        return
    
    if y_coord>0: # don't print for the reset bits
        print(f"Coordinate ({x_coord}, {y_coord}) successfully sent to register {mb_x_register}.")

def check_robot_cycle_complete(client) -> int:
    """
    check_robot_cycle_complete

    checks if the robot has finished its pick/place cycle
    
    :param client
    :return: 1=cycle is done; 0=cycle not done
    """
    try:
        result = client.read_input_registers(address = ROBOT_CYCLE_COMPLETE, count=1) # Read 1 register starting at 31
        if result.isError():
            print(f"Error reading input register 31")
            return

        register_value = result.registers[0]
        # print(f"RESULTS FROM READ REGISTER: {register_value} (binary: {bin(register_value)})")

        # Interpret the register value
        if register_value & 0b0001:  # Check if the '1' bit is set for off state
            print("Robot cycle is done.")
            return 1
        else:
            # print("No command sent.")
            return 0

    except Exception as e:
        print(f"Exception occurred while checking robot cycle_done register..")
        print(e)
        return 
    
def test():
    """
    test

    unused, test modbus connection and verify locations
    """
    client = initialize_modbus('tcp')
    # result = client.read_input_registers(31, 1) #, slave=1)

    result = client.read_input_registers(32)
    print(f"RESULTS FROM READ REGISTER {32} = {result.registers}")

    send_modbus_coords(client, 1, 10, 13)
    send_modbus_coords(client, 2, 11, 14)
    send_modbus_coords(client, 3, 12, 15)