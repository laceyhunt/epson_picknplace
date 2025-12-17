import modbus_fxns
client = modbus_fxns.initialize_modbus('tcp')
modbus_fxns.set_modbus_bit(client, modbus_fxns.START_COMMAND, 0)
modbus_fxns.set_modbus_bit(client, modbus_fxns.CONVEYOR_ON, 0)