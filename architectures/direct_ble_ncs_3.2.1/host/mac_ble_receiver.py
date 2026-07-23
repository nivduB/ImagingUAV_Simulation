import asyncio
from bleak import BleakScanner, BleakClient

DEVICE_NAME = "EPC901_TX"

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
DATA_UUID    = "12345678-1234-5678-1234-56789abcdef1"
CMD_UUID     = "12345678-1234-5678-1234-56789abcdef2"

received = bytearray()

def notification_handler(sender, data):
    received.extend(data)
    print(f"Received {len(data)} bytes, total={len(received)}")

async def main():
    print("Scanning for EPC901_TX...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name == DEVICE_NAME,
        timeout=20.0
    )

    if device is None:
        print("Could not find EPC901_TX. Make sure transmitter is flashed and advertising.")
        return

    print(f"Found {device.name}: {device.address}")

    async with BleakClient(device) as client:
        print("Connected")

        await client.start_notify(DATA_UUID, notification_handler)
        print("Subscribed to data notifications")

        input("Press Enter to send START capture command 0x01...")
        await client.write_gatt_char(CMD_UUID, bytes([0x01]), response=False)
        print("Sent START")

        input("Press Enter to send STOP/DUMP command 0x02...")
        await client.write_gatt_char(CMD_UUID, bytes([0x02]), response=False)
        print("Sent STOP/DUMP. Waiting for BLE data...")

        await asyncio.sleep(5)

        await client.stop_notify(DATA_UUID)

    print(f"Done. Total bytes received: {len(received)}")

    with open("epc901_dump.bin", "wb") as f:
        f.write(received)

    print("Saved to epc901_dump.bin")

asyncio.run(main())