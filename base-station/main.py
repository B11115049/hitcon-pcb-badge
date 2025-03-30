from backend_interface import BackendInterface
from ir_interface import IrInterface
from packet_processor import PacketProcessor
from config import Config
import asyncio


async def main():
    config = Config("config.yaml")
    backend = BackendInterface(config=config)
    ir = IrInterface()
    processor = PacketProcessor(backend=backend, ir=ir)
    processor.start()


if __name__ == "__main__":
    asyncio.run(main())
