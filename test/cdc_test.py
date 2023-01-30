from time import time
import random
import asyncio
# pip install pyserial-asyncio
from serial_asyncio import create_serial_connection
from serial_asyncio import SerialTransport
from multiprocessing import Pipe
from multiprocessing.connection import Connection

class InOutProtocol(asyncio.Protocol):
    __rx: Connection = None
    __buf: bytearray = bytearray()
    rx_cnt: int = 0
    tx_cnt: int = 0

    def set_pipe(self, pipe: Connection):
        if pipe.readable:
            self.__rx = pipe

    def connection_made(self, transport):
        self.transport = transport
        print('port opened', transport)
        transport.serial.rts = False  # You can manipulate Serial object via transport

    def data_received(self, data):
        if self.__rx != None:
            if self.__rx.poll(0):
                self.__buf.extend(self.__rx.recv_bytes())
            if bytearray(data) != self.__buf[0:len(data)]:
                print('error receiving')
                self.transport.close()
            else:
                self.__buf = self.__buf[len(data):]
        self.rx_cnt += len(data)

    def connection_lost(self, exc):
        print('port closed')
        self.transport.loop.stop()

    def pause_writing(self):
        print('pause writing')
        print(self.transport.get_write_buffer_size())

    def resume_writing(self):
        print(self.transport.get_write_buffer_size())
        print('resume writing')

refresh_delay = 0.1
send_delay = 0.01 # 0.01 minimum
size = 8192 # 4096 maximum for virtual com port emulator
deviation = 16
time_limit = 100
data_limit = 1024 * 1024 * 1
baudrate = 115200
iface = '/dev/ttyS1'

async def writing(transport: SerialTransport, protocol: InOutProtocol, pipe_rx: Connection, pipe_tx: Connection):
    protocol.set_pipe(pipe_rx)
    while protocol.tx_cnt < data_limit:
        await asyncio.sleep(send_delay)
        if protocol.tx_cnt - protocol.rx_cnt >= size: continue
        datalen = size - random.randint(0, deviation)
        if protocol.tx_cnt + datalen > data_limit:
            datalen = data_limit - protocol.tx_cnt
        data = random.randbytes(datalen)
        pipe_tx.send_bytes(data)
        protocol.tx_cnt += len(data)
        transport.write(data)

async def hypervisor(protocol: InOutProtocol):
    first_tm = time() * 1000
    max_diff = 0
    speed_tx = speed_rx = 0
    while True:
        await asyncio.sleep(refresh_delay)
        tm = time() * 1000 - first_tm
        tx = protocol.tx_cnt
        rx = protocol.rx_cnt
        diff = tx - rx
        if tm != 0:
            speed_tx = int(tx / (tm / 1000))
            speed_rx = int(rx / (tm / 1000))
        print(f'\rtime: {int(tm)} ms | tx = {tx} ({speed_tx} bit/s) rx = {rx} ({speed_rx} bit/s) diff {diff}    ', end = '')
        max_diff = max(max_diff, diff)
        if tm >= time_limit * 1000 or rx == data_limit:
            print('')
            print(f'max diff = {max_diff}')
            exit(0)

print(f'Starting with interval: {send_delay} s, packet size: {size - deviation}..{size} bytes, max speed: {int((size + deviation / 2) / send_delay)} bit/s')

loop = asyncio.get_event_loop()
coro = create_serial_connection(loop, InOutProtocol, iface, baudrate = baudrate)
io_transport, io_protocol = loop.run_until_complete(coro)
pipe_rx, pipe_tx = Pipe(duplex = False)
loop.create_task(writing(io_transport, io_protocol, pipe_rx, pipe_tx))
loop.run_until_complete(hypervisor(io_protocol))

loop.run_forever()
loop.close()

'''
Test report
stm32f413 virtual COM port  time: 53433 ms | tx = 10485760 (196241 bit/s) rx = 10485760 (196241 bit/s) max diff = 15336
ST-Link                     time: 91652 ms | tx =  1048576  (11440 bit/s) rx =  1048576  (11440 bit/s) max diff = 15358
PL2303                       time: 1441 ms | tx =    16377  (11360 bit/s) rx =     4096   (2841 bit/s) diff 12281 error receiving
CH340                       time: 91414 ms | tx =  1048576  (11470 bit/s) rx =  1048576  (11470 bit/s) max diff = 15358
CP2102                      time: 90922 ms | tx =  1048576  (11532 bit/s) rx =  1048576  (11532 bit/s) max diff = 16278
'''
