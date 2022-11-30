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

refresh_delay = 1
send_delay = 0.01 # 0.01 minimum
size = 4000 # 4096 maximum for virtual com port emulator
deviation = 16
time_limit = 32
data_limit = 1024 * 1024 * 10

async def writing(transport: SerialTransport, protocol: InOutProtocol, pipe_rx: Connection, pipe_tx: Connection):
    protocol.set_pipe(pipe_rx)
    while protocol.tx_cnt < data_limit:
        await asyncio.sleep(send_delay)
        datalen = size + random.randint(0, deviation)
        if protocol.tx_cnt + datalen > data_limit:
            datalen = data_limit - protocol.tx_cnt
        data = random.randbytes(datalen)
        pipe_tx.send_bytes(data)
        protocol.tx_cnt += len(data)
        transport.write(data)

async def hypervisor(protocols: list):
    first_tm = time()
    while True:
        await asyncio.sleep(refresh_delay)
        res = []
        tm = int(time() - first_tm)
        for i in protocols:
            tx = i.tx_cnt
            rx = i.rx_cnt
            res.append(f'tx = {tx} ({int(tx / tm)} bit/s) rx = {rx} ({int(rx / tm)} bit/s) diff {tx - rx}    ')
        print(f'\rtime: {tm} s', ' | '.join(res), end = '')
        if tm >= time_limit or rx == data_limit:
            print("")
            break

print(f'Starting with interval: {send_delay} s, packet size: {size}..{size + deviation} bytes, max speed: {int((size + deviation / 2) / send_delay)} bit/s')

loop = asyncio.get_event_loop()
coro = create_serial_connection(loop, InOutProtocol, '/dev/ttyS1', baudrate=115200)
io_transport, io_protocol = loop.run_until_complete(coro)
pipe_rx, pipe_tx = Pipe(duplex = False)
loop.create_task(writing(io_transport, io_protocol, pipe_rx, pipe_tx))
loop.run_until_complete(hypervisor([io_protocol]))

loop.run_forever()
loop.close()
