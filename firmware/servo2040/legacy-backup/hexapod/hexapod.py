from servo import ServoCluster, servo2040
import time
import json
import math

class Hexapod(object):

    # All measurements are in mm and are measured from the origin (0, 0, 64)
    LEG_OFFSETS = {
        "LEG_1": (164, 230, 64),
        "LEG_2": (280, 0, 64),
        "LEG_3": (164, -230, 64),
        "LEG_4": (-164, -230, 64),
        "LEG_5": (-280, 0, 64),
        "LEG_6": (-164, 230, 64)
    }

    # Offsets measurements are in degrees
    COXA_OFFSETS = {
        "LEG_1": -54.50947921755289,
        "LEG_2": 0.0,
        "LEG_3": 54.50947921755289,
        "LEG_4": -54.50947921755289,
        "LEG_5": 0.0,
        "LEG_6": 54.50947921755289
    }
    
    # Servo map for each leg by servo index. Always (coxa, femur, tibia)
    LEG_SERVO_MAP = {
         "LEG_1": (12, 13, 14),
         #"LEG_1": (15, 16, 17),
         "LEG_2": (6, 7, 8),
         "LEG_3": (0, 1, 2),
         #"LEG_4": (15, 16, 17),
         "LEG_4": (3, 4, 5),
         "LEG_5": (9, 10, 11),
         #"LEG_6": (3, 4, 5)
         "LEG_6": (15, 16, 17)
    }
    
    LEG_RIGHT_FRONT = (servo2040.SERVO_13, servo2040.SERVO_14, servo2040.SERVO_15)
    LEG_RIGHT_MIDDLE = (servo2040.SERVO_7, servo2040.SERVO_8, servo2040.SERVO_9)
    LEG_RIGHT_BACK = (servo2040.SERVO_1, servo2040.SERVO_2, servo2040.SERVO_3)
    LEG_LEFT_FRONT = (servo2040.SERVO_4, servo2040.SERVO_5, servo2040.SERVO_6)
    LEG_LEFT_MIDDLE = (servo2040.SERVO_10, servo2040.SERVO_11, servo2040.SERVO_12)
    LEG_LEFT_BACK = (servo2040.SERVO_16, servo2040.SERVO_17, servo2040.SERVO_18)
    
    SERVOS = 18
    SWEEP_RANGE = 90.0
    DEFAULT_SPEED = 0.1
    #CALIBRATION_FILE = "hexapod_calibration.json"
    CALIBRATION_FILE = "test_config.json"
    
    # Constants for inverse kinematics
    COXA_LENGTH = 41.0
    FEMUR_LENGTH = 116.0
    TIBIA_LENGTH = 183.0
    TOTAL_LENGTH = COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH
    BODY_RADIUS = 103.5 # Radius from the center of hexapod to coxa join in mm
    BODY_HEIGHT = -42.0
    
    def __init__(self) -> None:
        self.servos = ServoCluster(
            pio=0,
            sm=0,
            pins=list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
        )
        self.servo_positions = {}
        self.load_configuration(Hexapod.CALIBRATION_FILE)
        self.set_initial_positions()
        self.calibration_mode = False
        
        
    def set_initial_positions(self) -> None:
        """ Set all legs to initial positions before enabling them. """
        for i in range(Hexapod.SERVOS):
            if i % 3 == 0:
                self.servos.value(i, 0.0)
                self.servo_positions[i] = 0.0
            else:
                self.servos.value(i, 90.0)
                self.servo_positions[i] = 90.0
                
    def enable(self) -> None:
        """ Enable all legs. """
        self.servos.enable_all()
        time.sleep(2.0)
        self.load_configuration(filename='stand_stance.json')
        
    def disable(self) -> None:
        """ Disable all legs. """
        self.servos.disable_all()
        time.sleep(2.0)
        
    def phase_shift(self, step: int, tripod_1: list, tripod_2: list, angles_cache: dict, num_steps: int) -> None:
        """ Phase shift the legs, where one tripod moves while the other maintains contact. """
        for leg in tripod_1:
            start_angles, end_angles = angles_cache[leg]
            self.move_leg(leg, start_angles[0], start_angles[1], start_angles[2])
            
        for leg in tripod_2:
            start_angles, end_angles = angles_cache[leg]
            t = step / float(num_steps)
            interp_value = (1 - math.cos(math.pi * t)) / 2
            coxa_angle = start_angles[0] + (end_angles[0] - start_angles[0]) * interp_value
            femur_angle = start_angles[1] + (end_angles[1] - start_angles[1]) * interp_value
            tibia_angle = start_angles[2] + (end_angles[2] - start_angles[2]) * interp_value
            
            self.move_leg(leg, coxa_angle, femur_angle, tibia_angle)
            
    def calculate_ik(self, x: float, y: float, z: float, leg_id: str, body_center: tuple) -> tuple:
        """ Calculate the inverse kinematics for a leg. """
        x += body_center[0]
        y += body_center[1]
        z += body_center[2]
        
        radius = Hexapod.BODY_RADIUS if leg_id in ["LEG_2", "LEG_5"] else 120.378
            
        dist = math.sqrt(((Hexapod.LEG_OFFSETS[leg_id][0] + x) ** 2) + ((Hexapod.LEG_OFFSETS[leg_id][1]) + y) ** 2)
        y_target = Hexapod.LEG_OFFSETS[leg_id][1] + y
        x_target = Hexapod.LEG_OFFSETS[leg_id][0] + x
        z_target = Hexapod.BODY_HEIGHT + z
        
        H = dist - radius
        Z = z_target
        
        L = math.sqrt(H ** 2 + Z ** 2)
        coxa_rad = math.atan(y_target / x_target)
        
        try:
            # Clamp acos input to valid range to avoid math domain errors
            init_value = ((Hexapod.FEMUR_LENGTH ** 2) + (Hexapod.TIBIA_LENGTH ** 2) - (L ** 2)) / (2 * Hexapod.FEMUR_LENGTH * Hexapod.TIBIA_LENGTH)
            print(f"Initial Value: {init_value}")
            clamped_value = self.clamp(init_value, -1.0, 1.0)
            print(f"Clamped Value: {clamped_value}")
            tibia_rad = math.acos(clamped_value)
        except ValueError as e:
            print(f"ValueError in acos calculation: {e}")
            tibia_rad = 0.0
            
        alpha = math.atan(Z / H)
        
        try:
            # Clamp acos input to valid range
            init_value = ((L ** 2) + (Hexapod.FEMUR_LENGTH ** 2) - (Hexapod.TIBIA_LENGTH ** 2)) / (2 * L * Hexapod.FEMUR_LENGTH)
            beta = math.acos(
                self.clamp(init_value, -1.0, 1.0)
            )
        except ValueError as e:
            print(f"ValueError in acos calculation: {e}")
            beta = 0.0
            
        femur_rad = beta - alpha
        
        coxa_deg = math.degrees(coxa_rad) + Hexapod.COXA_OFFSETS[leg_id]
        femur_deg = math.degrees(femur_rad)
        tibia_deg = math.degrees(tibia_rad)
        
        coxa_deg = self.clamp(coxa_deg, -90, 90)
        femur_deg = self.clamp(femur_deg, -90, 90)
        tibia_deg = self.clamp(tibia_deg, -90, 90)
        
        print(f"Coxa Angle: {coxa_deg} - Femur Angle: {femur_deg} - Tibia Angle: {tibia_deg}")
        
        return coxa_deg, femur_deg, tibia_deg
    
    def sinusoidal_interpolation(self, legs: list, angles_cache: dict, num_steps: int=10, speed: float=DEFAULT_SPEED) -> None:
        """ Interpolates leg movement between positions over a number of steps. """
        tripods = [
            ("LEG_1", "LEG_3", "LEG_5"),
            ("LEG_2", "LEG_4", "LEG_6")
        ]
        for step in range(num_steps):
            for leg in tripods[0]:
                start_angles, end_angles = angles_cache[leg]
                
                # Interpolation factor
                t = step / float(num_steps)
                interp_value = (1 - math.cos(math.pi * t)) / 2
                
                # Intermediate angles
                coxa_angle = start_angles[0] + (end_angles[0] - start_angles[0]) * interp_value
                femur_angle = start_angles[1] + (end_angles[1] - start_angles[1]) * interp_value
                tibia_angle = start_angles[2] + (end_angles[2] - start_angles[2]) * interp_value
                
                self.move_leg(leg, coxa_angle, femur_angle, tibia_angle, speed)
                
            for leg in tripods[1]:
                start_angles, end_angles = angles_cache[leg]
                
                t = step / float(num_steps)
                interp_value = (1 - math.cos(math.pi * t)) / 2
                
                # Intermediate angles
                coxa_angle = start_angles[0] + (end_angles[0] - start_angles[0]) * interp_value
                femur_angle = start_angles[1] + (end_angles[1] - start_angles[1]) * interp_value
                tibia_angle = start_angles[2] + (end_angles[2] - start_angles[2]) * interp_value
                
                self.move_leg(leg, coxa_angle, femur_angle, tibia_angle, speed)
                
            time.sleep(1.0 / max(speed, 0.01))
            
        for step in range(num_steps):
            for leg in tripods[0]:
                start_angles, end_angles = angles_cache[leg]
                
                t = step / float(num_steps)
                interp_value = (1 - math.cos(math.pi * t)) / 2
                
                coxa_angle = end_angles[0] + (start_angles[0] - end_angles[0]) * interp_value
                femur_angle = end_angles[1] + (start_angles[1] - end_angles[1]) * interp_value
                tibia_angle = end_angles[2] + (start_angles[2] - end_angles[2]) * interp_value
                
                self.move_leg(leg, coxa_angle, femur_angle, tibia_angle, speed)
                
            for leg in tripods[1]:
                start_angles, end_angles = angles_cache[leg]
                
                t = step / float(num_steps)
                interp_value = (1 - math.cos(math.pi * t)) / 2
                
                coxa_angle = end_angles[0] + (start_angles[0] - end_angles[0]) * interp_value
                femur_angle = end_angles[1] + (start_angles[1] - end_angles[1]) * interp_value
                tibia_angle = end_angles[2] + (start_angles[2] - end_angles[2]) * interp_value
                
                self.move_leg(leg, coxa_angle, femur_angle, tibia_angle, speed)
                
            time.sleep(1.0 / max(speed, 0.01))
    
    def clamp(self, value: float, min_value: float, max_value: float) -> float:
        """ Clamp a value between a minimum and maximum value. """
        return max(min_value, min(max_value, value))
    
    def move_leg(self, leg_id: str, coxa_angle: float, femur_angle: float, tibia_angle: float, speed: float=DEFAULT_SPEED) -> None:
        """ Move a specific leg to the given x, y, z coordinates. """
        servos = Hexapod.LEG_SERVO_MAP[leg_id]
        
        self.servos.to_percent(servos[0], coxa_angle, -90, 90, speed)
        self.servos.to_percent(servos[1], femur_angle, -90, 90, speed)
        self.servos.to_percent(servos[2], tibia_angle, -90, 90, speed)
        
    def get_leg_angles(self, leg_id: str) -> tuple:
        """ Retrieve the current servo angles for the given leg. """
        return (
             self.servos.value(Hexapod.LEG_SERVO_MAP[leg_id][0]),
             self.servos.value(Hexapod.LEG_SERVO_MAP[leg_id][1]),
             self.servos.value(Hexapod.LEG_SERVO_MAP[leg_id][2])
        )
        
    def walk(self, x: float, y: float, z: float, speed: float=DEFAULT_SPEED, num_steps: int=10) -> None:
        """ Perform a walking gait with the hexapod using IK. """
        # Tripod 1: Legs 1, 4, 5
        # tripod 2: Legs 2, 3, 6
        tripods = [
            ("LEG_1", "LEG_3", "LEG_5"),
            ("LEG_2", "LEG_4", "LEG_6")
        ]
        all_legs = ["LEG_1", "LEG_2", "LEG_3", "LEG_4", "LEG_5", "LEG_6"]
        
        angles_cache = {}
        
        body_center = (0, 0, 64)
        # The outter range loop needs to be removed once testing is complete and user input is being used
        #for tripod in tripods:
            #for leg in tripod:
                #start_angles = self.get_leg_angles(leg)
                #end_angles = self.calculate_ik(x, y, z, leg)
                #angles_cache[leg] = (start_angles, end_angles)
         
        for leg in all_legs:
            start_angles = self.get_leg_angles(leg)
            end_angles = self.calculate_ik(x, y, z, leg, body_center)
            angles_cache[leg] = (start_angles, end_angles)
            
        self.sinusoidal_interpolation(all_legs, angles_cache, num_steps=num_steps, speed=speed)
        #for tripod in tripods:
            #self.sinusoidal_interpolation(tripod, angles_cache, num_steps=num_steps, speed=speed)
        #for step in range(num_steps):
            #self.phase_shift(step, tripods[0], tripods[1], angles_cache, num_steps)
            #time.sleep(1.0 / max(speed, 0.01))
                    
        
    def begin_calibration(self) -> None:
        """ Begin the calibration process. """
        self.calibration_mode = True
        print("Calibration mode enabled.")
        
    def process_calibration_packet(self, servo_index: int, value: float) -> None:
        """ Process incoming calibration packets to adjust servo positions. """
        if self.calibration_mode:
            print(f"Adjusting servo {servo_index} by {value}")
            current_value = self.servos.value(servo_index)
            new_value = current_value + value
            self.servos.value(servo_index, new_value)
            self.servo_positions[servo_index] = new_value
            #self.servos.load()
        
    def save_calibration(self) -> None:
        """ Finalize the calibration process by saving the current servo positions. """
        if self.calibration_mode:
            # Save the current servo positions to a JSON file
            for i in range(Hexapod.SERVOS):
                self.servo_positions[i] = self.servos.value(i)
            self.save_configuration()
            self.calibration_mode = False
            self.load_configuration()
    
    def save_configuration(self, filename: str = CALIBRATION_FILE) -> None:
        """ Save the current servo positions to a JSON file. """
        with open(filename, 'w') as config:
            json.dump(self.servo_positions, config)
        print(f"Configuration saved to {filename}")
        
    def load_configuration(self, filename: str = CALIBRATION_FILE) -> None:
        """ Load servo positions from a JSON file an apply them. """
        try:
            with open(filename, 'r') as file:
                config = json.load(file)
            for index, position in config.items():
                self.servos.value(int(index), position)
                self.servo_positions[int(index)] = position
            time.sleep(2.0)
            print(f"Configuration loaded from {filename}")
        #except FileNotFoundError:
        except Exception as e:
            print(f"Error occurred while loading configuration file: {e}.")
            for i in range(Hexapod.SERVOS):
                self.servo_positions[i] = 0.0
                
    def get_servo_values(self) -> list:
        """ Retreive the current values for all servos. """
        values = [self.servos.value(i) for i in range(Hexapod.SERVOS)]
        return values
                
            
if __name__ == "__main__":
    hexapod = Hexapod()
    hexapod.enable()
    
    #print(hexapod.get_servo_values())
    # Test increment and decrement
    #hexapod.walk(x=50.0, y=50.0, z=-30.0)
    hexapod.disable()
    # Test saving and loading configuration
    #hexapod.save_configuration("test_config.json")
    #hexapod.load_configuration("test_config.json")
        