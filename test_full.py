"""Full protocol test with virtual bus."""
import sys
import threading
import time

from can_bootloader_sim import CANBootloaderSimulator

sim = CANBootloaderSimulator(channel='test_bus', bitrate=125000)
t = threading.Thread(target=sim.run, daemon=True)
t.start()
time.sleep(0.5)

from can_bootloader import CANBootloader

bl = CANBootloader(interface='virtual', channel='test_bus', bitrate=125000)
result = "FAIL"
try:
    bl.connect()
    print("1. Connected!")
    
    bl.sync()
    print("2. Sync OK!")
    
    v, cmds = bl.get()
    print(f"3. Version=0x{v:02X}, Commands={len(cmds)}")
    
    cid = bl.get_id()
    print(f"4. Chip ID=0x{cid:04X}")
    
    data = bl.read_memory(0x08000000, 16)
    print(f"5. Read OK: {data.hex()}")
    
    # Write 256 bytes (full block)
    test_data = bytes(range(256))
    bl.write_memory(0x08000000, test_data)
    print("6. Write OK")
    
    # Read back
    data2 = bl.read_memory(0x08000000, 256)
    assert data2 == test_data, f"Readback mismatch at offset {next(i for i in range(256) if data2[i] != test_data[i])}"
    print("7. Readback OK")
    
    # Erase
    bl.erase_memory(global_erase=True)
    print("8. Erase OK")
    
    # Verify erased
    data3 = bl.read_memory(0x08000000, 16)
    assert data3 == bytes([0xFF]*16)
    print("9. Erase verify OK")
    
    result = "PASS"
except Exception as e:
    import traceback
    print(f"FAIL: {e}")
    traceback.print_exc()
finally:
    bl.disconnect()

print(f"\n*** TEST {result} ***")
sys.exit(0 if result == "PASS" else 1)
