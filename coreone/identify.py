import sys
sys.path.insert(0, 'klippy')
sys.path.insert(0, 'klippy/chelper')
import reactor as kreactor, serialhdl

PORT = "/dev/serial/by-id/usb-Klipper_stm32f427xx_XXXXXXXXXXXX-if00"
r = kreactor.Reactor()
ser = serialhdl.SerialReader(r)

def conn(e):
    try:
        ser.connect_uart(PORT, 115200)
        mp = ser.get_msgparser()
        ver, builds = mp.get_version_info()
        print("IDENTIFY OK")
        print("  MCU version :", ver)
        cfg = mp.get_constants()
        for k in ("MCU", "CLOCK_FREQ", "STATS_SUMSQ_BASE", "RECEIVE_WINDOW", "SERIAL_BAUD"):
            if k in cfg:
                print(f"  {k:12}: {cfg[k]}")
        # count of mcu commands in the data dictionary
        try:
            ncmd = len(mp.messages_by_id)
        except Exception:
            ncmd = "?"
        print("  cmd dict ids:", ncmd)
    except Exception as ex:
        print("IDENTIFY FAILED:", repr(ex))
    finally:
        r.end()

r.register_callback(conn)
try:
    r.run()
finally:
    try: ser.disconnect()
    except Exception: pass
