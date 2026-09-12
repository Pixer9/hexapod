from servo import ServoCluster, servo2040
import math
import time
import json

COXA_LENGTH = 41.0
FEMUR_LENGTH = 116.0
TIBIA_LENGTH = 183.0

CALIBRATION_FILE = "calibration.json"

class Leg(object):
    
    def __init__(self, mount_angle_deg, servo_indices) -> None:
        """
            mount_angle_deg: Mount angle (in degrees) measured from the positive x-axis.
                For example:
                    - Right front (Leg 1): 60
                    - Right middle (Leg 2): 0
                    - Right rear (Leg 3): -60
                    - Left rear (Leg 4): 240 (i.e. -120)
                    - Left middle (Leg 5): 180
                    - Left front (Leg 6): 120
            servo_indices: Tuple of three servo identifiers (coxa, femur, tibia) for this leg.
        """
        self.mount_angle = math.radians(mount_angle_deg)
        self.servos = servo_indices
        # Neutral parameters for leg extension and foot height
        self.d_desired = 150.0 # Effective extension for the femur + tibia chain.
        self.home_z = -40.0	# Default foot height relative to the body.
        # Calibration offsets (x, y, z) in mm.
        self.calibration_offset = (0.0, 0.0, 0.0)
        
    def set_calibration_offset(self, dx, dy, dz) -> None:
        """
            Set the calibration offset for this leg.
                dx, dy, dz: The additional offsets (in mm) to be added to the computed home position.
        """
        self.calibration_offset = (dx, dy, dz)
        
    def calculate_home_position(self) -> tuple:
        """
            Compute the leg's neutral (home) position based on its geometry, then add
            the calibration offsets.
            
            Computation:
                h = sqrt(d_desired**2 - home_z**2)
                r = COXA_LENGTH + h
                base_x = r * cos(mount_angle)
                base_y = r * sin(mount_angle)
                base_z = home_z
                
            Then, add calibration_offset (dx, dy, dz).
        """
        h = math.sqrt(max(0, self.d_desired**2 - self.home_z**2))
        r = COXA_LENGTH + h
        base_x = r * math.cos(self.mount_angle)
        base_y = r * math.sin(self.mount_angle)
        base_z = self.home_z
        dx, dy, dz = self.calibration_offset
        return base_x + dx, base_y + dy, base_z + dz
    
    def inverse_kinematics(self, x, y, z) -> None:
        """
            Compute the IK for a leg given a target (x, y, z) in global coordinates.
            First, rotate the (x, y) into the leg's local coordinate system using its mount angle.
            Then, solve for the joint angles.
        """
        # Rotate (x, y) by -mount_angle to get coordinates in the leg's local frame.
        cos_a = math.cos(self.mount_angle)
        sin_a = math.sin(self.mount_angle)
        x_local = x * cos_a + y * sin_a
        y_local = -x * sin_a + y * cos_a
        
        # Compute the coxa angle from the local (x, y) coordinates.
        theta_coxa = math.atan2(y_local, x_local)
        
        # Effective horizontal distance after subtracting the coxa length.
        horizontal_distance = math.sqrt(x_local**2 + y_local**2) - COXA_LENGTH
        # Distance in the plane for the femur and tibia.
        d = math.sqrt(horizontal_distance**2 + z**2)
        
        a = FEMUR_LENGTH
        b = TIBIA_LENGTH
        
        # Compute the femur angle using the law of cosines.
        cos_term = (a**2 + d**2 - b**2) / (2 * a * d)
        cos_term = max(-1.0, min(1.0, cos_term)) # Clamp to valid range
        angle_offset = math.acos(cos_term)
        theta_femur = math.atan2(z, horizontal_distance) + angle_offset
        
        # Compute the tibia angle using the law of cosines.
        cos_tibia = (a**2 + b**2 - d**2) / (2 * a * b)
        cos_tibia = max(-1.0, min(1.0, cos_tibia))
        theta_tibia = math.pi - math.acos(cos_tibia)
        
        # Convert angles from radians to degrees.
        return math.degrees(theta_coxa), math.degrees(theta_femur), math.degrees(theta_tibia)
    
    def move(self, target_x, target_y, target_z, speed, servo_cluster) -> None:
        """
            Compute the IK for the given target and command the servos.
            
            Parameters:
                - target_x, target_y, target_z: The desired global foot position.
                - speed: Speed factor for the servo transition.
                - servo_cluster: The shared ServoCluster object to command.
        """
        coxa_angle, femur_angle, tibia_angle = self.inverse_kinematics(target_x, target_y, target_z)
        coxa_index, femur_index, tibia_index = self.servos
        
        # Debug output: print target position and calculated servo angles.
        print(f"Leg mounted at {math.degrees(self.mount_angle):.1f}:")
        print(f"  Target (x, y, z): ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
        print(f"  IK angles -> Coxa: {coxa_angle:.2f}, Femur: {femur_angle:.2f}, Tibia: {tibia_angle:.2f}")
        
        # Send commands to the servos. (Assume servos range is -90 to 90)
        servo_cluster.to_percent(coxa_index, coxa_angle, -90, 90, speed)
        servo_cluster.to_percent(femur_index, femur_angle, -90, 90, speed)
        servo_cluster.to_percent(tibia_index, tibia_angle, -90, 90, speed)
        
class Hexapod(object):
    
    def __init__(self) -> None:
        """
            Initialize the ServoClister for 18 servos.
            
        """
        self.servos = ServoCluster(
            pio=0,
            sm=0,
            pins=list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
        )
        # Initialize legs.
        # For now, we only add leg 1 (right front leg) with a mount angle of 60
        self.legs = []
        # Rght side legs
        leg1 = Leg(mount_angle_deg=60, servo_indices=(servo2040.SERVO_13, servo2040.SERVO_14, servo2040.SERVO_15))      
        leg2 = Leg(mount_angle_deg=0, servo_indices=(servo2040.SERVO_7, servo2040.SERVO_8, servo2040.SERVO_9))   
        leg3 = Leg(mount_angle_deg=-60, servo_indices=(servo2040.SERVO_1, servo2040.SERVO_2, servo2040.SERVO_3))
        # Left side legs.
        leg4 = Leg(mount_angle_deg=240, servo_indices=(servo2040.SERVO_4, servo2040.SERVO_5, servo2040.SERVO_6))
        leg5 = Leg(mount_angle_deg=180, servo_indices=(servo2040.SERVO_10, servo2040.SERVO_11, servo2040.SERVO_12))
        leg6 = Leg(mount_angle_deg=120, servo_indices=(servo2040.SERVO_16, servo2040.SERVO_17, servo2040.SERVO_18))

        self.legs.extend([leg1, leg2, leg3, leg4, leg5, leg6])
    
    def save_calibration(self, filename=CALIBRATION_FILE) -> None:
        """
            Save the calibration offsets and home_z values for all legs to a JSON file.
        """
        data = {}
        for i, leg in enumerate(self.legs):
            data[f"leg_{i+1}"] = {
                "calibration_offset": leg.calibration_offset,
                "home_z": leg.home_z
            }
        try:
            with open(filename, "w") as f:
                json.dump(data, f)
            print(f"Calibration data save to {filename}")
        except Exception as e:
            print(f"Error saving calibration: {e}")
        
    def load_calibration(self, filename=CALIBRATION_FILE) -> None:
        """
            Load calibration data from a JSON file and apply it to the legs.
        """
        try:
            with open(filename, "r") as f:
                data = json.load(f)
            for i, leg in enumerate(self.legs):
                key = f"leg_{i+1}"
                if key in data:
                    leg.calibration_offset = tuple(data[key].get("calibration_offset", (0.0, 0.0, 0.0)))
                    leg.home_z = data[key].get("home_z", -40.0)
            print(f"Calibration data loaded from {filename}")
        except Exception as e:
            print(f"Error loading calibration: {e}")
            
    def initialize(self) -> None:
        """
            Initialize all legs by moving them to their calculated home positions.
        """
        self.load_calibration()
        for leg in self.legs:
            home_x, home_y, home_z = leg.calculate_home_position()
            leg.move(home_x, home_y, home_z, speed=0.1, servo_cluster=self.servos)
        time.sleep(1)
    
    def enable(self) -> None:
        """ Enable all legs. """
        self.servos.enable_all()
        time.sleep(2.0)
        #self.load_configuration(filename='stand_stance.json')
        
    def disable(self) -> None:
        """ Disable all legs. """
        self.servos.disable_all()
        time.sleep(2.0)
    
    def calibrate_leg(self, leg_index, dx, dy, dz) -> None:
        """
            Adjust the calibration offset for a single leg.
                leg_index: 0-based index of the leg in self.legs.
                dx, dy, dz: Offsets (in mm) to be added to that leg's home position.
        """
        if 0 <= leg_index < len(self.legs):
            self.legs[leg_index].set_calibration_offset(dx, dy, dz)
            print(f"Leg {leg_index+1} calibrated with offset ({dx}, {dy}, {dz})")
        else:
            print("Invalid leg index.")
            
    def move_body(self, offset_x, offset_y, offset_z) -> None:
        """
            Given an (x, y, z) offset (e.g., from joystick input), update each leg's target position.
        
            The new target position for each leg is computed as:
                target = home_position + (body offset)
            
            For now, this is a simple translation applied uniformly.
        """
        for leg in self.legs:
            home_x, home_y, home_z = leg.calculate_home_position()
            target_x = home_x + offset_x
            target_y = home_y + offset_y
            target_z = home_z + offset_z
            leg.move(target_x, target_y, target_z, speed=0.1, servo_cluster=self.servos)
    
    def execute_joystick_gait(self, joystick_offset, steps_per_cycle=50, cycle_time=2.0, step_length=50.0, lift=30.0) -> None:
        """
            Execute one complete gait cycle using the trigate algorithm while incorporating a joystick.
            The joystick_offset (a tuple: (ox, oy, oz)) is added to each leg's calibrated home position.
            
            For each half-cycle:
                - Swing legs: move forward relative to (home + joystick_offset) with a prabolic lift.
                - Stance legs: slie backward relative to (home + joystick_offset).
            
            At the start and end of the cycle, the gait offsets are zero, so the leg is exactly at home.
            
            Legs are divided as:
                Group 1: Legs 2, 4, 6 (indices 1, 3, 5)
                Group 2: Legs 1, 3, 5 (indices 0, 2, 4)
        """
        group1 = [self.legs[1], self.legs[3], self.legs[5]]
        group2 = [self.legs[0], self.legs[2], self.legs[4]]
        
        for step in range(steps_per_cycle):
            t = step / steps_per_cycle
            if t < 0.5:
                t_phase = t * 2
                for leg in group1:
                    home_x, home_y, home_z = leg.calculate_home_position()
                    base_x = home_x + joystick_offset[0]
                    base_y = home_y + joystick_offset[1]
                    base_z = home_z + joystick_offset[2]
                    target_x = base_x
                    target_x = base_x
                    target_y = base_y + (step_length / 2) * math.sin(math.pi * t_phase)
                    target_z = base_z + lift * math.sin(math.pi * t_phase)
                    leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                for leg in group2:
                    home_x, home_y, home_z = leg.calculate_home_position()
                    base_x = home_x + joystick_offset[0]
                    base_y = home_y + joystick_offset[1]
                    base_z = home_z + joystick_offset[2]
                    target_x = base_x
                    target_y = base_y - (step_length / 2) * math.sin(math.pi * t_phase)
                    target_z = base_z
                    leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
            else:
                t_phase = (t - 0.5) * 2
                for leg in group2:
                    home_x, home_y, home_z = leg.calculate_home_position()
                    base_x = home_x + joystick_offset[0]
                    base_y = home_y + joystick_offset[1]
                    base_z = home_z + joystick_offset[2]
                    target_x = base_x
                    target_y = base_y + (step_length / 2) * math.sin(math.pi * t_phase)
                    target_z = base_z + lift * math.sin(math.pi * t_phase)
                    leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                for leg in group1:
                    home_x, home_y, home_z = leg.calculate_home_position()
                    base_x = home_x + joystick_offset[0]
                    base_y = home_y + joystick_offset[1]
                    base_z = home_z + joystick_offset[2]
                    target_x = base_x
                    target_y = base_y - (step_length / 2) * math.sin(math.pi * t_phase)
                    target_z = base_z
                    leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
            time.sleep(cycle_time/steps_per_cycle)
        for leg in self.legs:
            home_x, home_y, home_z = leg.calculate_home_position()
            target_x = home_x + joystick_offset[0]
            target_y = home_y + joystick_offset[1]
            target_z = home_z + joystick_offset[2]
            leg.move(target_x, target_y, target_z, speed=0.2, servo_cluster=self.servos)
        time.sleep(0.5)
        
    def simulate_trigate_walk(self, cycles=3, steps_per_cycle=50, cycle_time=2.0, step_length=50.0, lift=30.0) -> None:
        """
            Simulate a trigate walking gait.
            
            The legs are split into two groups:
                - Group 1: legs 2, 4, and 6 (right middle, left rear, left front)
                - Group 2: legs 1, 3, and 5 (right front, right rear, left middle)
                
            During one half of the cycle, one group swings (moves along a parabolic trajectory) while the other
            holds its home (stance) position.
            
            Parameters:
                cycles: number of complete gait cycles.
                steps_per_cycle: resolution (more steps = smoother movement)
                cycle_time: total time for one complete cycle.
                step_length: total forward/backward displacement (mm) for the swing.
                lift: maximum vertical lift (mm) during the swing phase.
        """
        # Define our groups based on leg orders:
        # Our legs lists: [leg1, leg2, leg3, leg4, leg5, leg6]
        group1 = [self.legs[1], self.legs[3], self.legs[5]]
        group2 = [self.legs[0], self.legs[2], self.legs[4]]
        
        for cycle in range(cycles):
            print(f"Cycle {cycle+1}/{cycles}")
            # t goes from 0 to 1 over the cycle
            for step in range(steps_per_cycle):
                t = step / steps_per_cycle
                
                if t < 0.5:
                    # First half-cycle: Group 1 swings, Group 2 in stance
                    t_phase = t * 2 # normalized phase 0->1
                    for leg in group1:
                        # Swing trajectory: from (home - (0, step_length)) to home
                        home_x, home_y, home_z = leg.calculate_home_position()
                        # The swing leg's target moves forward relative to the body
                        target_x = home_x
                        target_y = home_y + (step_length / 2) * math.sin(math.pi * t_phase)
                        target_z = home_z + lift * math.sin(math.pi * t_phase)
                        leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                    for leg in group2:
                        home_x, home_y, home_z = leg.calculate_home_position()
                        target_x = home_x
                        target_y = home_y - (step_length / 2) * math.sin(math.pi * t_phase)
                        target_z = home_z
                        leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                
                else:
                    # Second half-cycle: Group 2 swings, Group 1 in stance
                    t_phase = (t - 0.5) * 2
                    for leg in group2:
                        home_x, home_y, home_z = leg.calculate_home_position()
                        target_x = home_x
                        target_y = home_y + (step_length / 2) * math.sin(math.pi * t_phase)
                        target_z = home_z + lift * math.sin(math.pi * t_phase)
                        leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                    for leg in group1:
                        home_x, home_y, home_z = leg.calculate_home_position()
                        target_x = home_x
                        target_y = home_y - (step_length / 2) * math.sin(math.pi * t_phase)
                        target_z = home_z
                        leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
                time.sleep(cycle_time/steps_per_cycle)
            # At the end of each cycle, return all legs to their home positions.
            for leg in self.legs:
                home_x, home_y, home_z = leg.calculate_home_position()
                leg.move(home_x, home_y, home_z, speed=0.2, servo_cluster=self.servos)
            time.sleep(0.5)
            
    def simulate_leg_walk(self, cycles=3, steps_per_cycle=50, cycle_time=2.0, step_length=50.0, lift=30.0) -> None:
        """
            Simulate a walking step for the first leg using a parabolic trajectory.
            
            The trajectory for each cycle is defined as follows:
                - The foot moves linearly in the y direction from a rear (negative offset) to a front x direction
                    relative to its home position.
                - The z coordinate follows a parabola such that the foot lifts off the ground mid-swing.
                
            Parameters to tweak:
                - cycles: Number of step cycles.
                - steps_per_cycle: Resolution of the movement (more steps = smoother motion)
                - cycle_time: Total time for on complete cycle (in seconds)
                - step_length: Total displacement in the y direction during a step (mm).
                - lift: Maximum additional lift in z (mm) during the swing phase.
        """
        # For now, simulate only for the first leg.
        leg = self.legs[0]
        home_x, home_y, home_z = leg.calculate_home_position()
        print(f"Home position: x={home_x:.2f}, y={home_y:.2f}, z={home_z:.2f}")
        
        for cycle in range(cycles):
            print(f"Cycle {cycle+1}/{cycles}")
            for step in range(steps_per_cycle):
                t = step / steps_per_cycle
                # Linear interpolation for y offset: move from -step_length/ 2 to +step_legnth/2
                relative_y = -step_length/2 + step_length * t
                # Parabolic z offset: maximum lift at t=0.5
                # Equation: z_offset = lift * 4 * t * (1 - t)
                z_offset = lift * 4 * t * (1 - t)
                for leg in self.legs:
                    home_x, home_y, home_z = leg.calculate_home_position()
                    target_x = home_x
                    target_y = home_y + relative_y
                    target_z = home_z + z_offset
                    leg.move(target_x, target_y, target_z, speed=cycle_time/steps_per_cycle, servo_cluster=self.servos)
            
            # Return legs to home between cycles
            for leg in self.legs:
                home_x, home_y, home_z = leg.calculate_home_position()
                leg.move(home_x, home_y, home_z, speed=0.2, servo_cluster=self.servos)
            time.sleep(0.5)
            
    