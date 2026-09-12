from hexapod.hexapod import Hexapod
import json
import time

class CommandProcessor(object):
    
    def __init__(self, hexapod: Hexapod) -> None:
        self.hexapod = hexapod
        
    def process_command(self, json_packet: str) -> None:
        """ Process incoming JSON command. """
        try:
            command_data = json.loads(json_packet)
            command_id = command_data.get("command_id")
            data_id = command_data.get("data_id")
            payload = command_data.get("payload", {})
            
            if command_id == "MOVE":
                self.hexapod.set_leg_positions(data_id, payload)
            elif command_id == "CALIBRATE":
                if data_id == "BEGIN":
                    self.hexapod.begin_calibration()
                elif data_id == "ADJUST":
                    servo_index = payload.get("servo_index", 0)
                    value = payload.get("value", 0.0)
                    self.hexapod.process_calibration_packet(servo_index, value)
                elif data_id == "COMPLETE":
                    self.hexapod.save_calibration()
            elif command_id == "REQUEST":
                # Handle data request (i.e., IMU readings)
                pass
        #except json.JSONDecodeError as error:
        except Exception as error:
            print(f"Error occurred while processing command: {error}")
            
if __name__ == "__main__":
    hexapod = Hexapod()
    processor = CommandProcessor(hexapod=hexapod)
    
    # Test some command packets
    processor.process_command(json.dumps({"command_id": "CALIBRATE", "data_id": "BEGIN"}))
    
    print("Calibration mode active.")
    
    
    while True:
        servo_input = input("Enter servo index to calibrate(0-17) or 'x' to exit: ").strip()
        
        if servo_input.lower() == 'x':
            print("Exiting calibration.")
            break
        
        try:
            servo_index = int(servo_input)
            if servo_index < 0 or servo_index > 17:
                print("Invalid servo index. Please enter a number between 0 and 17.")
                continue
        except ValueError:
            print("Invalid input. Please entere a number between 0 and 17.")
            continue
        
        print(f"Calibrating servo {servo_index}.")
        print("Enter 'u' to increment, 'd' to decrement, and 'Enter' to select another servo, or 'x' to exit.")
        
        while True:
            user_input = input("Command: ").strip()
            
            if user_input.lower() == 'u':
                processor.process_command(json.dumps({"command_id": "CALIBRATE", "data_id": "ADJUST", "payload": {"servo_index": servo_index, "value": 1.0}}))
                print(f"Incremented servo {servo_index}")
            elif user_input.lower() == 'd':
                processor.process_command(json.dumps({"command_id": "CALIBRATE", "data_id": "ADJUST", "payload": {"servo_index": servo_index, "value": -1.0}}))
                print(f"Decremented servo {servo_index}")
            elif user_input.lower() == 'x':
                print("Exiting calibration.")
                processor.process_command(json.dumps({"command_id": "CALIBRATE", "data_id": "COMPLETE"}))
            elif user_input == '':
                print("Returning to servo selection.")
                break
            
            time.sleep(0.1)
