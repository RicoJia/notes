# Modbus server (TCP)
from pymodbus.server import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext

from argparse import ArgumentParser

PORT=5020

def run_async_server():
    print('Modbus server started on localhost port ', PORT)
    # number of registers
    nreg = 200
    # initialize data store
    store = ModbusSlaveContext(
        #[15, 15 ...]
        di=ModbusSequentialDataBlock(0, [15]*nreg),
        co=ModbusSequentialDataBlock(0, [16]*nreg),
        hr=ModbusSequentialDataBlock(0, [17]*nreg),
        ir=ModbusSequentialDataBlock(0, [18]*nreg))
    context = ModbusServerContext(slaves=store, single=True)

    # initialize the server information
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'APMonitor'
    identity.ProductCode = 'APM'
    identity.VendorUrl = 'https://apmonitor.com'
    identity.ProductName = 'Modbus Server'
    identity.ModelName = 'Modbus Server'
    identity.MajorMinorRevision = '3.0.2'

    # TCP Server
    StartTcpServer(context=context, host='localhost',\
                   identity=identity, address=("localhost", PORT))

def run_simple_client():
    print('Modbus simple client is on')
    from pymodbus.client import ModbusTcpClient
    client = ModbusTcpClient("localhost", port=PORT)
    if client.connect():
        #TODO Remember to remove
        print(f'Connected')
        client.write_coil(1, True)
        result = client.read_coils(1,1)
        print(result.bits[0])
        client.close()
    else:
        print(f'Not connected')

def run_holding_reg_read_write_client():
    # Modbus client
    from pymodbus.client import ModbusTcpClient as ModbusClient
    from pymodbus.constants import Endian
    from pymodbus.payload import BinaryPayloadBuilder
    from pymodbus.payload import BinaryPayloadDecoder
    import time

    print('Start Modbus Client')
    client = ModbusClient(host='127.0.0.1', port=PORT)
    reg=0; address=0

    # initialize data
    data = [0.1,1.1,2.1,3.1,4.1]

    for i in range(10):
        print('-'*5,'Cycle ',i,'-'*30)
        time.sleep(1.0)

        # increment data by one
        for i,d in enumerate(data):
            data[i] = d + 1

        # write holding registers (40001 to 40005)
        print('Write',data)
        builder = BinaryPayloadBuilder(byteorder=Endian.Big,\
                                        wordorder=Endian.Little)
        for d in data:
            builder.add_16bit_int(int(d))
        payload = builder.build()
        result  = client.write_registers(int(reg), payload,\
                    skip_encode=True, unit=int(address))

        # read holding registers
        rd = client.read_holding_registers(reg,len(data)).registers
        print('Read',rd)

    client.close()

if __name__ == "__main__":
    argparser = ArgumentParser()
    argparser.add_argument("--server", default=False, action="store_true")
    args = argparser.parse_args()
    if args.server:
        run_async_server()
    else:
        # run_simple_client()
        run_holding_reg_read_write_client()
        