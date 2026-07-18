# TCA6408A I2C GPIO expander — fan/MMU-power enable for the xBuddy extension.
#
# On the extension the three fan power switches and the MMU power are gated by a
# TCA6408A on I2C2 (addr 0x40 8-bit = 0x20 7-bit). Klipper's [fan] can't reach an
# I2C expander, so this module drives the enable bits at connect (mirroring Prusa's
# enable_fans(): write Output reg then set all pins as outputs). With the fans
# enabled here, the actual speed is just the hardware PWM on the TIM3 pins.
#
# Bit map (from src/puppy/xbuddy_extension/hal.cpp): b5=fan1_en, b4=fan2_en,
# b3=fan3_en, b2=mmu_power. Registers: 1=Output, 3=Config (0 = output).
#
# Example config:
#   [tca6408a extension_fans]
#   i2c_mcu: extension
#   i2c_bus: i2c2_PB10_PB13   # hardware I2C2 (SDA on PB13, AF4); or software pins
#   #i2c_address: 0x20     # default
#   enable_bits: 0x38      # b5|b4|b3 = all three fans on
#
# Validated on hardware 2026-06-07 (hardware I2C2 PB10/PB13 on the H503).
import logging
from . import bus

TCA6408A_REG_OUTPUT = 1
TCA6408A_REG_CONFIG = 3

class TCA6408A:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.i2c = bus.MCU_I2C_from_config(config, default_addr=0x20,
                                           default_speed=100000)
        # enable_bits accepts hex (e.g. 0x38) or decimal
        self.enable_bits = int(config.get('enable_bits', '0x38'), 0)
        if not 0 <= self.enable_bits <= 255:
            raise config.error("enable_bits must be 0-255 in section '%s'"
                               % (config.get_name(),))
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self):
        # set the desired output levels, then configure all pins as outputs
        self.i2c.i2c_write([TCA6408A_REG_OUTPUT, self.enable_bits])
        self.i2c.i2c_write([TCA6408A_REG_CONFIG, 0x00])
        # Confirm the expander actually acknowledged -- a software-I2C write to
        # the wrong pins/missing device NACKs silently, so verify the read-back.
        try:
            resp = self.i2c.i2c_read([TCA6408A_REG_OUTPUT], 1)
            rb = bytearray(resp['response'])[0]
            if rb != self.enable_bits:
                logging.warning("tca6408a: read-back 0x%02x != written 0x%02x",
                                rb, self.enable_bits)
        except Exception as e:
            logging.warning("tca6408a: read-back failed (expander not "
                            "responding?): %r", e)
        logging.info("tca6408a: enabled outputs 0x%02x", self.enable_bits)

def load_config_prefix(config):
    return TCA6408A(config)
