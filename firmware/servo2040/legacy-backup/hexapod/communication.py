from plasma import WS2812
from servo import servo2040
from hexapod.hexapod import Hexapod
import uselect
import machine
import time
import json
import sys
import gc

I2C_SLAVE_ADDRESS = 0x42
BRIGHTNESS = 0.4
UPDATES = 50
SPEED = 5

led_bar = WS2812(servo2040.NUM_LEDS, 1, 0, servo2040.LED_DATA)
led_bar.start()

class CommunicationHandler(object):
    
    def __init__(self, i2c_bus=None, serial_port=None) -> None:
        self.i2c_bus = i2c_bus
        self.serial_port = serial_port
        
    def send_packet(self, packet: dict) -> None:
        json_packet = json.dumps(packet)
        if self.serial_port:
            self.serial_port.write(json_packet.encode('utf-8'))
        elif self.i2c_bus:
            self.i2c_bus.write(json_packet.encode('utf-8'))
            
    def receive_packet(self) -> dict:
        if self.serial_port:
            received = self.serial_port.read()
        elif self.i2c_bus:
            received = self.i2c_bus.read()
        else:
            return {}
        
        try:
            return json.loads(received.decode())
        except json.JSONDecodeError:
            print("Failed to decode JSON packet.")
            return {}
        
    def listen(self) -> None:
        """ Actively listen for Serial Packet transmissions - This needs SERIOUS refinement. """
        while True:
            gc.collect()
            serialList = uselect.select([sys.stdin], [], [], 0.01)
            
            if serialList[0]:
                try:
                    inCommand = sys.stdin.readline().strip()
                    
                    if len(inCommand) > 0:
                        command = json.loads(inCommand)
                        print(command)
                        response_packet = {
                            "status": "success",
                            "message": "Command received and processed."
                        }
                        send_response(json.dumps(response_packet))
                except Exception as e:
                    error_response = {
                        "status": "error",
                        "message": f"Invalid packet: {str(e)}"
                    }
                    send_response(json.dumps(error_response))

def send_response(response):
    sys.stdout.write(response)

def indicate_command_received():
    offset = 0.0
    for _ in range(int(UPDATES * 2)):
        offset += SPEED / 1000.0
        
        for i in range(servo2040.NUM_LEDS):
            hue = float(i) / servo2040.NUM_LEDS
            led_bar.set_hsv(i, hue + offset, 1.0, BRIGHTNESS)
            
        time.sleep(1.0 / UPDATES)
        
    led_bar.clear()
    
def receive_data(usb_uart):
    if usb_uart.any():
        data = usb_uart.read()
        if data:
            data_str = data.decode('utf-8').strip()
            print(f"Received from Pi: {data_str}")
            return data_str
    return None
    
if __name__ == "__main__":
    com = CommunicationHandler(serial_port=True)
    com.listen()