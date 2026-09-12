from servo import Servo, ServoCluster, servo2040
import time

def ensure_enabled(func):
    def wrapper(self, *args, **kwargs):
        self.enable()
        result = func(self, *args, **kwargs)
        time.sleep(0.5)
        return result
    return wrapper

class Leg(object):
    
    def __init__(self, coxa: Servo, femur: Servo = None, tibia: Servo = None) -> None:
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia
        
    def set_initial_position(self, coxa_position: float, femur_position: float, tibia_position: float) -> None:
        """ Set initial positions for the servos before enabling them. """
        self.coxa.value(coxa_position)
        if self.femur is not None:
            self.femur.value(femur_position)
        if self.tibia is not None:
            self.tibia.value(tibia_position)
        
    def enable(self) -> None:
        """ Enable all servos on a leg. """
        self.coxa.enable()
        if self.femur is not None:
            self.femur.enable()
        if self.tibia is not None:
            self.tibia.enable()
        
    def disable(self) -> None:
        """ Disable all servos on a leg. """
        self.coxa.disable()
        if self.femur is not None:
            self.femur.disable()
        if self.tibia is not None:
            self.tibia.disable()
        
    def get_position(self) -> tuple:
        """ Return the current positions of the servos. """
        return (self.coxa.value(), self.femur.value(), self.tibia.value())
    
    def set_position(self, coxa_position: float, femur_position: float, tibia_position: float) -> None:
        """ Set the positions of all servos in the leg. """
        self.coxa.value(coxa_position)
        self.femur.value(femur_position)
        self.tibia.value(tibia_position)

        
        
class Hexapod(object):
    
    LEG_RIGHT_FRONT = (servo2040.SERVO_13, servo2040.SERVO_14, servo2040.SERVO_15)
    LEG_RIGHT_MIDDLE = (servo2040.SERVO_7, servo2040.SERVO_8, servo2040.SERVO_9)
    LEG_RIGHT_BACK = (servo2040.SERVO_1, servo2040.SERVO_2, servo2040.SERVO_3)
    LEG_LEFT_FRONT = (servo2040.SERVO_4, servo2040.SERVO_5, servo2040.SERVO_6)
    LEG_LEFT_MIDDLE = (servo2040.SERVO_10, servo2040.SERVO_11, servo2040.SERVO_12)
    LEG_LEFT_BACK = (servo2040.SERVO_16, servo2040.SERVO_17, servo2040.SERVO_18)
    
    SERVOS = 18
    SWEEP_RANGE = 90.0
    STEPS = 10
    STEP_INTERVAL = 0.5
    
    
    def __init__(self) -> None:
        self.servos = ServoCluster(
            pio=0,
            sm=0,
            pins=list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
        )

        self.set_initial_positions()
        
    def set_initial_positions(self) -> None:
        """ Set all legs to initial positions before enabling them. """
        for i in range(Hexapod.SERVOS):
            if i % 3 == 0:
                self.servos.value(i, 0.0)
            else:
                self.servos.value(i, 90.0)
        
    def enable(self) -> None:
        """ Enable all legs. """
        self.servos.enable_all()
        time.sleep(2.0)
        
    def disable(self) -> None:
        """ Disable all legs. """
        self.servos.disable_all()
        time.sleep(2.0)
         
    def set_single_servo(self, servo_index: int, servo_value: int) -> None:
        """ Set the value of a single servo. """
        if servo_index >= 0 and servo_index <= Hexapod.SERVOS - 1:
            if servo_value >= -Hexapod.SWEEP_RANGE and servo_value <= Hexapod.SWEEP_RANGE:
                self.servos.value(servo_index, servo_value)
            else:
                raise ValueError(f"Servo value must be between {-Hexapod.SWEEP_RANGE} and {Hexapod.SWEEP_RANGE}.")
        else:
            raise ValueError(f"Servo index must be between {0} and {Hexapod.SERVOS - 1}.")
        
    def move_all_legs(self, target_positions: list, steps: int, delay: float) -> None:
        """
            Move all legs towards their target positions simultaneously in steps.
            target_positions should be a list of tuples, where each tuple represents
            the target_positions (coxa, femur, tibia) for each leg.
        """
        current_positions = [leg.get_position() for leg in self.legs]
        increments = [
            (
                (target[0] - current[0]) / steps,
                (target[1] - current[1]) / steps,
                (target[2] - current[2]) / steps
            )
            for target, current in zip(target_positions, current_positions)
        ]
        
        for step in range(steps):
            for leg, (inc_coxa, inc_femur, inc_tibia), (cur_coxa, cur_femur, cur_tibia) in zip(self.legs, increments, current_positions):
                new_coxa = cur_coxa + (inc_coxa * step)
                new_femur = cur_femur + (inc_femur * step)
                new_tibia = cur_tibia + (inc_tibia * step)
                leg.set_position(new_coxa, new_femur, new_tibia)
            time.sleep(delay)
            
        for leg, (coxa_target, femur_target, tibia_target) in zip(self.legs, target_positions):
            leg.set_position(coxa_target, femur_target, tibia_target)
                
    def set_leg(self, leg_index, coxa_value, femur_value, tibia_value) -> None:
        """ Calibrate a specific leg by setting each servo to its desired position. """
        if 0 <= leg_index < len(self.legs):
            self.legs[leg_index].set_position(coxa_value, femur_value, tibia_value)
            
    def set_all_legs(self, coxa_value, femur_value, tibia_value, speed=1.0) -> None:
        """ Set all legs to the same position. """
        for leg in self.legs:
            leg.set_position(coxa_value, femur_value, tibia_value)
            
    def maintenance_stance(self) -> None:
        """ Move all legs to the stand stance. """
        target_positions = [
            (0.0, 90.0, 90.0),
            (0.0, 90.0, 90.0),
            (0.0, 90.0, 90.0),
            (0.0, 90.0, 90.0),
            (0.0, 90.0, 90.0),
            (0.0, 90.0, 90.0),
        ]
        self.move_all_legs(target_positions, steps=50, delay=0.01)
          
    def stand(self) -> None:
        """ Move legs to standing position. """
        target_positions = [
            (-10.0, 90.0, 45.0),
            (5.0, 75.0, 45.0),
            (5.0, 75.0, 45.0),
            (5.0, 75.0, 45.0),
            (5.0, 75.0, 45.0),
            (5.0, 75.0, 45.0),
        ]
        self.move_all_legs(target_positions, steps=50, delay=0.01)
        
    def process_command(self, command):
        """ Process serial commands from the Pi 5. """
        pass
    
    
        
    