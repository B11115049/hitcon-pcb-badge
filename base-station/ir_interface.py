"""
My implementation
I expect badge will send ir packet to base station actively.

USB Serial Commands:

Nesseary Fields:
Preamble - 8 bytes. 0xD5 0x55 0x55 0x55 0x55 0x55 0x55 0x55

PacketType - 1 byte
    * 0x01 for QueueTxBufferRequest
    * 0x81 for QueueTxBufferResponse
    * 0x02 for RetrieveRxBufferRequest 
    * 0x82 for RetrieveRxBufferResponse
    * 0x03 for GetStatusRequest
    * 0x83 for GetStatusResponse
    * 0x04 for SendStatusRequest
    * 0x84 for SendStatusResponse
    * 0x05 for PopRxBufferRequest (for duplex mode)
    * 0x85 for PopRxBufferResponse

Optional Fields:
PacketSequence - 1 byte. A random byte to denote which response maps to which request.
    The response should have the same packet sequence as the request.

PacketSize - 1 byte.

PacketData - x bytes

Api of each packet type:
QueueTxBufferRequest: (Computer to Badge)
1 byte - PacketSequence.
1 byte - PacketSize.
x bytes - The entire buffer data to queue for transmit.

QueueTxBufferResponse: (Badge to Computer)
1 byte - PacketSequence.
1 byte - 0x01 for buffer full, 0x02 for success.

RetrieveRxBufferRequest: (Computer to Badge)
None

RetrieveRxBufferResponse: (Badge to Computer)
1 byte - PacketSize. 0x00 for buffer empty.
x bytes for the entire buffer received.

GetStatusRequest: (Computer to Badge)
None

GetStatusResponse: (Badge to Computer)
1 byte - 0x01 for failure, 0x02 for success.
1 byte - Status:
            * 0x01: Set for not ready to receive, all received buffers has been populated with received packet.
            * 0x02: Set for ready to receive, there are empty received buffers exist on badge.

SendStatusRequest: (Badge to Computer)
1 byte - The status, same as above.

SendStatusResponse: (Computer to Badge)
None

PopRxBufferRequest: (Badge to Computer)
1 byte - PacketSequence.
1 byte - PacketSize.
x bytes - The entire buffer data to queue for transmit.

PopRxBufferResponse: (Computer to Badge)
1 byte - PacketSequence.
1 byte - 0x01 for failure, 0x02 for success.

"""

import serial
import enum
from config import Config
import asyncio
import time
from dataclasses import dataclass
from typing import Optional

# PacketType enum
class PT(enum.Enum):
    QTQ = b'\x01' # QueueTxBufferRequest
    QTR = b'\x81' # QueueTxBufferResponse
    RRQ = b'\x02' # RetrieveRxBufferRequest
    RRR = b'\x82' # RetrieveRxBufferResponse
    GSQ = b'\x03' # GetStatusRequest
    GSR = b'\x83' # GetStatusResponse
    SSQ = b'\x04' # SendStatusRequest
    SSR = b'\x84' # SendStatusResponse
    PRQ = b'\x05' # PopRxBufferRequest
    PRR = b'\x85' # PopRxBufferResponse

# Packet content format
@dataclass
class Packet:
    # No include packet type so that caller need to have the information beforehand 
    seq: Optional[bytes] = None
    packet_size_raw: Optional[int] = None
    is_success: Optional[bytes] = None
    status: Optional[bytes] = None
    payload: Optional[bytes] = None

    # For_read_packet_and_parse function, fields must be in the order of the packet.
    # Packet_size_raw existance implys payload existance.
    # Str in the list means the field with same name in the class will be used to parse the data.
    packet_info = {
        PT.QTR: ['seq', 'is_success'],
        PT.RRR: ['packet_size_raw'],
        PT.GSR: ['is_success', 'status'],
        PT.SSQ: ['status'],
        PT.PRQ: ['seq', 'packet_size_raw'],
    }


    def __bytes__(self):
        data = b''
        for b in (self.seq, self.packet_size_raw, self.is_success, self.status, self.payload):
            if b == None:
                continue
            if type(b) == int:
                data += bytes([b])
            else:
                data += b
        return data


class IrInterface:
    preamble = b'\xD5\x55\x55\x55\x55\x55\x55\x55'
    success = b'\x02'
    failure = b'\x01'

    def __init__(self, config: Config):
        self.port = config.get(key="usb_port")
        self.baudrate = config.get(key="usb_baudrate")
        self.timeout = config.get(key="usb_timeout")
        self.failure_try = config.get(key="usb_failure_try")
        self.failure_wait = config.get(key="usb_failure_wait")
        self.packet_que_max = config.get(key="packet_que_max")
        self.duplex = config.get(key="duplex")
        self.seq_set = set()
        self.current_seq = 0
        self.waiting_que_table = dict([(e, dict()) for e in (PT.QTR,)] + 
            [(e, asyncio.Queue(maxsize=self.packet_que_max)) for e in (PT.RRR, PT.GSR)])
        self.write_lock = asyncio.Lock()
        self.read_lock = asyncio.Lock()
        self.half_duplex_lock = asyncio.Lock()
        self.badge_status: Packet = None

        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
        except serial.SerialException as e:
            print(f"Failed to initialize serial port: {e}")
            raise

    async def trigger_send_packet(self, data: bytes, packet_type = PT.QTQ) -> bool | Packet:
        # Triggers the ir interface to send a packet.
        # Returns True if sent successfully, False otherwise.
        for _ in range(self.failure_try):          
            try:
                with self.serial as s:
                    result: bool | Packet = await self._write_packet(data, packet_type)

                if type(result) == Packet and result.is_success == IrInterface.success:
                    return result
            
                if result == True:
                    return True
                
            except Exception as e:
                print(f"Error sending packet: {e}")
            
            await asyncio.sleep(self.failure_wait)

        return False
    

    async def get_next_packet(self) -> bytes:
        # Wait until the next packet arrives, then return its raw data.    
        for _ in range(self.failure_try):
            try:
                with self.serial as s:
                    return await self._read_packet() # May idle
                
            except Exception as e:
                print(f"Error getting packet: {e}")

            await asyncio.sleep(self.failure_wait)

        return b''
    

    async def _write_packet(self, data: bytes, packet_type = PT.QTQ, lock = True) -> bool | Packet:
        # will return reponse if exist
        match packet_type:
            case PT.QTQ: # QueueTxBufferRequest
                return await self._write_packet_and_wait_for_response(data, packet_type=PT.QTQ, lock=lock)
            
            case PT.RRQ | PT.SSR: # RetrieveRxBufferRequest | SendStatusResponse
                await self._write(packet_type.value, lock=lock)
                return True 
            
            case PT.GSQ: # GetStatusRequest
                return await self._write_packet_and_wait_for_response(data, packet_type=PT.GSQ, lock=lock)

            case PT.PRR: # PopRxBufferResponse
                await self._write(packet_type.value + data, lock=lock)
                return True 
            
        return False
    
    async def _write_packet_and_wait_for_response(self, data: bytes, packet_type = PT.QTQ, lock = True) -> bool | Packet:
        result = None
        
        match packet_type:
            case PT.QTQ: # QueueTxBufferRequest
                seq = IrInterface._get_seq()
                p = Packet(seq=seq, packet_size_raw=len(data).to_bytes(), payload=data)
                self.waiting_que_table[PT.QTR][seq] = asyncio.Queue(maxsize=1)
                await self._write(packet_type.value + p.__bytes__(), lock=lock)

                try:
                    result = await asyncio.wait_for(self.waiting_que_table[PT.QTR][seq].get(), timeout=self.timeout)

                    if result:
                        return result.is_success == IrInterface.success
                finally:
                    self._remove_seq(seq)
                    self.waiting_que_table[PT.QTR].pop(seq, None)

            case PT.GSQ: # GetStatusRequest
                await self._write(packet_type.value, lock=lock)
                result = await asyncio.wait_for(self.waiting_que_table[PT.GSR].get(), timeout = self.timeout)

                if result and result.is_success == IrInterface.success:
                    return result

        return False
    

    async def _read_packet(self) -> bytes:
        while True:
            async with self._get_lock(read=True):
                await self._read_until_preamble()
                packet_type = await self._read(1)
                p = await self._read_packet_and_parse(Packet.packet_info[PT(packet_type)])

                match PT(packet_type):
                    case PT.QTR: # QueueTxBufferResponse
                        if p.seq in self.waiting_que_table[PT.QTR] and not self.waiting_que_table[PT.QTR][p.seq].full():
                            self.waiting_que_table[PT.QTR][p.seq].put_nowait(p)

                    case PT.RRR: # RetrieveRxBufferResponse
                        return p.payload
                    
                    case PT.GSR: # GetStatusResponse
                        if not self.waiting_que_table[PT.GSR].full():
                            self.waiting_que_table[PT.GSR].put_nowait(p)

                    case PT.SSQ: # SendStatusRequest
                        self.badge_status = p.status
                        await self._write(PT.SSR.value, lock=True)

                    case PT.PRQ: # PopRxBufferRequest
                        success = p.seq and p.packet_size_raw and len(p.payload) == int.from_bytes(p.packet_size_raw)
                        asyncio.create_task(self._write_packet(p.seq + (IrInterface.success if success else IrInterface.failure), PT.PRR.value))   

                        if success:   
                            return p.payload

            await asyncio.sleep(0) # Yield control to allow other tasks to run


    async def _read_packet_and_parse(self, fields: list) -> Packet:
        data = await self._read(len(fields))
        packet = Packet()
        offset = 0

        for f in fields:
            setattr(packet, f, data[offset])
            offset += 1

        if  packet.packet_size_raw and int.from_bytes(packet.packet_size_raw) > 0:
            packet.payload = await self._read(int.from_bytes(packet.packet_size_raw))

        return packet


    def _get_lock(self, read: bool) -> asyncio.Lock:
        if read:
            return self.read_lock if self.duplex else self.half_duplex_lock
        return self.write_lock if self.duplex else self.half_duplex_lock


    def _get_seq(self) -> bytes:
        for _ in range(256):
            self.current_seq = (self.current_seq + 1) % 256
            if self.current_seq.to_bytes() not in self.seq_set:
                break
        else:
            raise RuntimeError("No available sequence numbers")

        self.seq_set.add(self.current_seq.to_bytes())
        return self.current_seq.to_bytes()


    def _remove_seq(self, seq: bytes):
        if seq in self.seq_set:
            self.seq_set.remove(seq)


    async def _read_until_preamble(self, wait = 0, timeout = None, lock = False) -> bool:
        plen = len(IrInterface.preamble)
        timer = time.time()
        byte = b''
        
        if plen <= 0:
            return True
        
        async def loop() -> bool:
            while True:
                byte += await self._read(plen - len(byte))

                if byte == IrInterface.preamble:
                    return True

                while IrInterface.preamble[0] in byte:
                    i = byte.find(IrInterface.preamble[0])

                    if not byte[i:] in IrInterface.preamble:
                        byte = byte[i + 1:] 
                    else:
                        byte = byte[i:]
                        break
                else:
                    byte = b''
                
                if timeout != None and time.time() - timer >= timeout:
                    return False

                await asyncio.sleep(wait)

        if lock:
            async with self._get_lock(read=True):
                return await loop()
            
        return await loop()


    async def _write(self, data: bytes, lock = False) -> None:
        if len(data) <= 0:
            return

        if lock:
            async with self._get_lock(read=False):
                await asyncio.wait_for(asyncio.to_thread(self.serial.write, IrInterface.preamble + data), timeout=None)
        else:
            await asyncio.wait_for(asyncio.to_thread(self.serial.write, IrInterface.preamble + data), timeout=None)
        

    async def _read(self, size: int, lock = False) -> bytes:
        if size <= 0:
            return b''
        
        if lock:
            async with self._get_lock(read=True):
                return await asyncio.wait_for(asyncio.to_thread(self.serial.read, size), timeout=None)
        
        return await asyncio.wait_for(asyncio.to_thread(self.serial.read, size), timeout=None)
                
        
    def __del__(self):
        if self.serial.is_open:
            self.serial.close()
    
    
        

