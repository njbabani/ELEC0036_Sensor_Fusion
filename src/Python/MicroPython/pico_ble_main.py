import bluetooth
import ubinascii
from ble_advertising import advertising_payload
from micropython import const
from machine import Pin
import time

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_INDICATE_DONE = const(20)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)
_FLAG_INDICATE = const(0x0020)

GENERIC_ACCESS_UUID = bluetooth.UUID(0x1800) # Generic Access Service UUID
SERVICE_CHAR = (bluetooth.UUID(0x2AF9), _FLAG_READ,)  # Using the Generic Level characteristic UUID
GENERIC_SERVICE = (GENERIC_ACCESS_UUID, (SERVICE_CHAR,),)

class BLEAdvertiser:
    def __init__(self, ble, name=""):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle,),) = self._ble.gatts_register_services([GENERIC_SERVICE])
        self._connections = set()
        if len(name) == 0:
            name = 'Pico %s' % ubinascii.hexlify(self._ble.config('mac')[1],':').decode().upper()
        print('Device name %s' % name)
        self._payload = advertising_payload(
            name=name, services=[GENERIC_ACCESS_UUID]
        )
        self._advertise()

    def _irq(self, event, data):
        # Track connections for potential future use.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_INDICATE_DONE:
            conn_handle, value_handle, status = data


    def _advertise(self, interval_us=20000): #50 Hz advertisment
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

def demo():
    ble = bluetooth.BLE()
    advertiser = BLEAdvertiser(ble)
    counter = 0
    led = Pin('LED', Pin.OUT)
    while True:
        led.toggle()
        time.sleep_ms(1000)
        counter += 1

if __name__ == "__main__":
    demo()
