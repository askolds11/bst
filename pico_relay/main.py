from machine import SPI, Pin, I2C
from nrf24l01.nrf24l01 import NRF24L01, POWER_0, POWER_3, SPEED_250K
from sht4x.sht4x import SHT4X
import random
import struct
import utime
from micropython import const

SENSOR_NUM = const(1009)

# Responder pause between receiving data and checking for further packets.
_RX_POLL_DELAY = const(15)
# Responder pauses an additional _RESPONER_SEND_DELAY ms after receiving data and before
# transmitting to allow the (remote) initiator time to get into receive mode. The
# initiator may be a slow device. Value tested with Pyboard, ESP32 and ESP8266.
_RESPONDER_SEND_DELAY = const(10)

spi = SPI(0, sck=Pin(6), mosi=Pin(7), miso=Pin(4))
cfg = {"spi": spi, "csn": 14, "ce": 17}


# Addresses are in little-endian format. They correspond to big-endian
# 0xf0f0f0f0e1, 0xf0f0f0f0d2
pipes = (b"\xe1\xf0\xf0\xf0\xf0", b"\xd2\xf0\xf0\xf0\xf0")


def read_sensor() -> tuple[int, int]:
    temperature = random.randint(0, 1023)
    humidity = random.randint(0, 1023)

    return temperature, humidity

def setup_nrf24l01() -> NRF24L01:
    csn = Pin(cfg["csn"], mode=Pin.OUT, value=1)
    ce = Pin(cfg["ce"], mode=Pin.OUT, value=0)
    spi = cfg["spi"]
    nrf = NRF24L01(spi, csn, ce, payload_size=16)

    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.set_power_speed(POWER_0, SPEED_250K)

    return nrf

testty = []

def receive(nrf: NRF24L01, testt):
    if nrf.any():
        print("Received something")
        while nrf.any():
            buf = nrf.recv()
            sensor_num, millis, temperature, humidity = struct.unpack("iiff", buf)
            print("received:", sensor_num, millis, temperature, humidity)
            utime.sleep_ms(_RX_POLL_DELAY)

        # Give initiator time to get into receive mode.
        utime.sleep_ms(_RESPONDER_SEND_DELAY)
        nrf.stop_listening()

        if (sensor_num, millis) in testt:
            # print("Already received this!")
            nrf.start_listening()
            pass
        else:
            # remove oldest, add newest
            if (len(testt) == 3):
                testt = testt[1:]
            testt.append((sensor_num, millis))

            try:
                # print("Sending", sensor_num, millis)
                nrf.send(struct.pack("ii", sensor_num, millis))
            except OSError:
                pass
            # print("sent response")

            # relay to gateway
            nrf.open_tx_pipe(pipes[1])
            nrf.open_rx_pipe(1, pipes[0])

            send_attempts = 0
            success = False

            while send_attempts < 10 and not success:
                send_attempts += 1

                # Send message
                print("Sending: ", sensor_num, millis)
                try:
                    data = struct.pack("iiff", sensor_num, millis, temperature, humidity)
                    nrf.send(data)
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

                    sensor_num_r, got_millis_r = struct.unpack("ii", buf)

                    if (sensor_num_r == sensor_num and millis == got_millis_r):
                        print("Succesfully sent!")

                    success = True

            # switch back to receiving
            nrf.open_tx_pipe(pipes[0])
            nrf.open_rx_pipe(1, pipes[1])
            nrf.start_listening()
            print("Listening...")

def send_nrf24l01(nrf: NRF24L01, sht: SHT4X):
    '''Sends temperature and humidity'''
    nrf.stop_listening()
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, pipes[0])

    (temperature, humidity) = sht.measurements

    send_attempts = 0
    success = False

    while send_attempts < 10 and not success:
        send_attempts += 1

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
    
    # Switch back to receive
    nrf.open_tx_pipe(pipes[0])
    nrf.open_rx_pipe(1, pipes[1])
    nrf.start_listening()
    return success

i2c = I2C(1, sda=Pin(2), scl=Pin(3))  # Correct I2C pins for RP2040
sht = SHT4X(i2c)
last_send = 0
sleep_time = 100

nrf = setup_nrf24l01()
nrf.start_listening()
nrf.print_info()
while True:
    receive(nrf, testty)
    if (utime.ticks_diff(utime.ticks_ms(), last_send) > sleep_time):
        if (send_nrf24l01(nrf, sht)):
            sleep_time = random.randint(59000, 61000)
        else:
            sleep_time = random.randint(19000, 21000)

        last_send = utime.ticks_ms()
        