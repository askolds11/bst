"""Test for nrf24l01 module.  Portable between MicroPython targets."""

import machine
import random
import usys
import ustruct as struct
import utime
from machine import Pin, SPI, SoftSPI, I2C
from nrf24l01.nrf24l01 import NRF24L01, POWER_0, POWER_1, SPEED_250K
from sht4x.sht4x import SHT4X
from micropython import const

SENSOR_NUM = const(1001)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 14, "ce": 17}

# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")

def lightsleep(time_ms: int):
    machine.lightsleep(time_ms)
    # utime.sleep_ms(time_ms)

def deepsleep(time_ms: int):
    machine.lightsleep(time_ms)
    # utime.sleep_ms(time_ms)

def setup_nrf24l01() -> NRF24L01:
    '''
    Initializes NRF24L01
    
    Returns:
    Initialized NRF24L01 object
    '''

    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    nrf = NRF24L01(spi, csn, ce, payload_size=16)

    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.set_power_speed(POWER_0, SPEED_250K)

    return nrf

# def is_radio_clear(nrf: NRF24L01) -> bool:
#     '''Check if radio is clear'''

#     radio_clear = False
#     retries = 0
    
#     # Try to find clear moment until radio is clear
#     # or fail if more than 10 tries
#     while not radio_clear and retries < 10:
#         nrf.start_listening()
#         if (nrf.any()):
#             nrf.stop_listening()
#             retries += 1
#             sleep_time = random.randint(5, 20)
#             lightsleep(sleep_time)
#         else:
#             nrf.stop_listening()
#             radio_clear = True

#     # If failed to find clear moment, return False
#     if (radio_clear == False):
#         return False
    
#     return True

def send_nrf24l01(nrf: NRF24L01, sht: SHT4X):
    '''Sends temperature and humidity'''
    
    (temperature, humidity) = sht.measurements

    send_attempts = 0
    success = False

    while send_attempts < 10 and not success:
        send_attempts += 1

        # Check if radio is clear, if not, try later
        # if not is_radio_clear(nrf):
        #     print("Radio was not clear!")
        #     sleep_time = random.randint(50, 100)
        #     lightsleep(sleep_time)
        #     pass

        # Send message
        millis = utime.ticks_ms()
        print("Sending: ", SENSOR_NUM, millis, temperature, humidity)
        try:
            nrf.send(struct.pack("iiff", SENSOR_NUM, millis, temperature, humidity))
        except OSError:
            utime.sleep_ms(2000)
            print("Send failed")
            pass

        # start listening again
        nrf.start_listening()
        print("Listening")

        # wait for response, with 250ms timeout
        start_time = utime.ticks_ms()
        timeout = False
        while not nrf.any() and not timeout:
            if utime.ticks_diff(utime.ticks_ms(), start_time) > 250:
                timeout = True

        # If timeout, try again.
        # Do not sleep because already waited from timeout
        if timeout:
            print("Timeout")
            nrf.stop_listening()
            pass
        else:
            print("Received")
            # recv packet
            buf = nrf.recv()
            nrf.stop_listening()

            sensor_num, got_millis = struct.unpack("ii", buf)

            if (sensor_num == SENSOR_NUM and millis == got_millis):
                print("Succesfully sent! Delay:", utime.ticks_diff(utime.ticks_ms(), got_millis))

            success = True
    return success
    
i2c = I2C(1, sda=Pin(2), scl=Pin(3))  # Correct I2C pins for RP2040
sht = SHT4X(i2c)

while True:
    nrf = setup_nrf24l01()
    sent_success = send_nrf24l01(nrf, sht)
    sleep_time = 0
    if sent_success:
        sleep_time = random.randint(59000, 61000)
    else:
        sleep_time = random.randint(19000, 21000)
    nrf.power_down()
    deepsleep(sleep_time)