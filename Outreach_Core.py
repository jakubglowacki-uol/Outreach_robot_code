import os
import sys
import time
import math
import threading
try:
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

try:
    from utils.UR_Functions import URfunctions as URControl
    HAS_UR = True
except ImportError:
    HAS_UR = False
    URControl = None

try:
    from robotiq.robotiq_gripper import RobotiqGripper
    HAS_GRIPPER = True
except ImportError:
    HAS_GRIPPER = False
    RobotiqGripper = None

try:
    from PyLabware.devices.ika_rct_digital import RCTDigitalHotplate
    HAS_HOTPLATE = True
except ImportError:
    HAS_HOTPLATE = False
    RCTDigitalHotplate = None

 
 
# === Robot Configuration ===
ROBOT_POSITIONS = {
    "Front_Home_Position": [0.8972518444061279, -1.7109610042967738, 2.0420382658587855, -0.3741963666728516, 0.9695882201194763, 3.176609992980957],
    "Left_Home_Position": [0.8972518444061279, -1.7109610042967738, 2.0420382658587855, -0.3741963666728516, 0.9695882201194763, 3.176609992980957],

    "Pipette_Holder_P1": [1.0766570568084717, -1.1974879068187256, 2.0551164785968226, -0.8901921075633545, 1.0556089878082275, 3.1875925064086914],
    "Above_Pipette_Holder_P1": [1.0771057605743408, -1.4582339239171525, 1.8416951338397425, -0.41710932672534184, 1.0550282001495361, 3.1898393630981445],

    "Pipette_Holder_P2": [0.9993770122528076, -1.1837261778167267, 2.016986672078268, -0.8671066922000428, 0.9953725337982178, 3.190044641494751],
    "Above_Pipette_Holder_P2": [0.9997725486755371, -1.4240033638528367, 1.8198745886432093, -0.43088992059741216, 0.9948712587356567, 3.192204475402832],

    "Pipette_Holder_P3": [0.9221579432487488, -1.1521427196315308, 1.9579084555255335, -0.8283539575389405, 0.9272118210792542, 3.155653476715088],
    "Above_Pipette_Holder_P3": [0.9225919246673584, -1.3910482686809083, 1.7424023787127894, -0.37528903902087407, 0.9265671968460083, 3.1579205989837646],
    
    "Pipette_Above_SVH2_P1": [0.9839673042297363, -1.7851759395995082, 2.3823002020465296, -0.6267851156047364, 1.961118221282959, 3.1616086959838867],
    "Pipette_Just_In_SVH2_P1": [0.9838283061981201, -1.7396818600096644, 2.4181421438800257, -0.7082246106914063, 1.9612091779708862, 3.1612794399261475],
    "Pipette_In_SVH2_P1": [0.9905354976654053, -1.5334394735148926, 2.5041282812701624, -1.0010349315455933, 1.9684126377105713, 3.159672975540161],
    
    "Pipette_Above_SVH2_P2": [0.8659958243370056, -1.9033719501891078, 2.4592140356646937, -0.5844482344440003, 1.8430041074752808, 3.1655118465423584],
    "Pipette_Just_In_SVH2_P2": [0.8877210021018982, -1.8313237629332484, 2.4840994516955774, -0.6816542905620118, 1.864976167678833, 3.1644530296325684],
    "Pipette_In_SVH2_P2": [0.8871084451675415, -1.612101217309469, 2.5887988249408167, -1.005936936741211, 1.864998698234558, 3.1630024909973145],
    
    "Pipette_Above_SVH2_P3": [0.7447376251220703, -2.02695431331777, 2.5231905619250696, -0.5243099492839356, 1.7214906215667725, 3.1692605018615723],
    "Pipette_Just_In_SVH2_P3": [0.7818475961685181, -1.948510309258932, 2.5731874147998255, -0.6792735022357483, 1.7580888271331787, 3.145226240158081],
    "Pipette_In_SVH2_P3": [0.7809306383132935, -1.6896139583983363, 2.6902971903430384, -1.055556671028473, 1.7578468322753906, 3.14363694190979],

    "Pipette_Near_Hotplate_Vial": [1.5263832807540894, -1.7597104511656703, 1.93235952058901, -0.20739682138476567, 1.889049768447876, 3.1421141624450684],
    "Pipette_Above_Hotplate_Vial": [1.574676513671875, -1.5646590388244768, 1.9610326925860804, -0.4306951326182862, 1.4871240854263306, 3.1553125381469727],
    "Pipette_In_Hotplate_Vial": [1.5746296644210815, -1.547818725263216, 1.9889138380633753, -0.47540028512988286, 1.4872093200683594, 3.155164957046509],
    
    "Pipette_Above_Pipette_Bin": [-0.6214655081378382, -1.6172100506224574, -1.9516651630401611, -2.657548566857809, -0.700897518788473, 3.0920045375823975],
    "Pipette_In_Pipette_Bin": [-0.6215084234820765, -1.934718748132223, -2.1650853157043457, -2.126490732232565, -0.7020323912249964, 3.09183669090271],


}

MOVEMENT_PARAMS = {
    "speed": 0.25,
    "acceleration": 0.1,
    "blending": 0.01,
}


# Choices
# 1. Full synthesis using Mettler Toledo Quantos
# 2. Copper chloride and piroctone olamine solutions already prepared

# Choices once syngthesis is ready
# 1. Synthesise and measure one sample of piroctone olamine
# 2. Measure particle size distribution over time
# 3. Measure impact of copper chloride concentration on particle size distribution


# # === IKA RCT Digial Hotplate Configuration

def find_hotplate_port():
    """
    Detects and returns the serial port of an IKA RCT hotplate.
    Tries all ports until one responds successfully.
    """
    if not HAS_SERIAL or not HAS_HOTPLATE:
        print("Warning: serial or hotplate driver not available. Skipping lookup.")
        return None, None
        
    ports = serial.tools.list_ports.comports()
    plate = None

    for port in ports:
        # Windows ports: COMx
        if sys.platform.startswith('win'):
            if port.device.startswith('COM'):
                if 'IKA' in port.description or 'USB' in port.description:
                    print(f"Found IKA device on {port.device} ({port.description})")
                    return port.device, None
                print(f"Found possible device on {port.device} ({port.description})")
                return port.device, None
        
        # Linux ports: /dev/ttyUSBx or /dev/ttyACMx
        elif sys.platform.startswith('linux'):
            print(f"Trying port: {port.device} ({port.description})")
            try:
                serial_port = port.device
                # Create the hotplate instance
                plate = RCTDigitalHotplate(
                    device_name="IKA RCT Digital",
                    connection_mode="serial",
                    address=None,
                    port=serial_port
                )

                plate.connect()
                plate.initialize_device()
            except:
                print("No IKA hotplate on this serial port found", "\n")
            
        # macOS ports: /dev/tty.* or /dev/cu.*
        elif sys.platform.startswith('darwin'):
            if port.device.startswith('/dev/tty.') or port.device.startswith('/dev/cu.'):
                print(f"Trying port: {port.device} ({port.description})")
                try:
                    serial_port = port.device
                    plate = RCTDigitalHotplate(
                        device_name="IKA RCT Digital",
                        connection_mode="serial",
                        address=None,
                        port=serial_port
                    )
                    plate.connect()
                    plate.initialize_device()
                except:
                    print("No IKA hotplate on this serial port found", "\n")
                
    return port.device, plate   # THIS PART CAUSES THE ERROR, KEEP IT AS IS

# === Helper Functions ===
def degreestorad(angles_deg):
    return [angle * math.pi / 180 for angle in angles_deg]


class OutreachDemo:
    """Educational wrapper exposing only add_indicator/add_acid/add_base.

    Usage (for students):
        demo = OutreachDemo()
        demo.initialize()
        demo.add_indicator()
        demo.add_acid(0.25)
        demo.add_base(4.5)
        demo.dispose()  # optional; also called automatically
    """

    def __init__(
        self,
        robot_ip="192.168.0.2",
        robot_port=30003,
        gripper_port=63352,
        simulation=False,
    ):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.gripper_port = gripper_port
        self.simulation_mode = bool(simulation)
        self.robot = None
        self.gripper = None
        self.plate = None
        self._initialized = False
        self._busy_lock = threading.Lock()
        self._is_busy = False

    @property
    def is_busy(self):
        return self._is_busy

    # --- Lifecycle ---
    def initialize(self):
        """Connect robot, gripper, hotplate; move home and start stirring."""
        if self.simulation_mode:
            self._initialized = True
            print("[SIMULATION] Initialization complete (no hardware connections attempted).")
            return

        # Check for missing hardware dependencies if not in simulation mode
        if not HAS_UR:
            raise ImportError("URControl dependency (ur-rtde) is missing. Switch to simulation mode or install dependencies.")
        if not HAS_GRIPPER:
            print("Warning: RobotiqGripper dependency missing. Gripper movements will fail.")
        
        if self.robot is None:
            self.robot = URControl(ip=self.robot_ip, port=self.robot_port)

        if self.gripper is None and HAS_GRIPPER:
            self.gripper = RobotiqGripper()
            self.gripper.connect(self.robot_ip, self.gripper_port)

        if self.plate is None:
            self.plate = find_hotplate_port()[1]
            print("Retrieved plate:", self.plate)

        # Initial position
        self._operate_gripper(0)
        self._move_robot(ROBOT_POSITIONS["Front_Home_Position"])
        print("Starting in home position", "\n")

        # Turn on Hotplate stirring
        if self.plate:
            self.plate.set_speed(500)
            self.plate.start_stirring()
            time.sleep(1)
            print(f"Speed: {self.plate.get_speed()} rpm")

        self._initialized = True

    def dispose(self):
        """Safely stop stirring/cleanup. Called automatically on destruction."""
        if self.simulation_mode:
            self._initialized = False
            print("[SIMULATION] Dispose complete (no hardware cleanup required).")
            return

        try:
            if self.plate:
                self.plate.stop_stirring()
                print("Stirring stopped.")
            self._initialized = False
        except Exception as exc:
            print(f"Warning: failed to stop plate cleanly: {exc}")

    def __del__(self):
        # Automatic cleanup if object is garbage-collected
        try:
            self.dispose()
        except Exception:
            pass

    # --- Public student-facing methods ---
    def add_indicator(self):
        self._run_exclusive(
            self._add_indicator_impl,
            "Robot is currently busy. Wait for the current action to finish.",
        )

    def add_acid(self, amount_ml):
        self._run_exclusive(
            lambda: self._add_acid_impl(amount_ml),
            "Robot is currently busy. Wait for the current action to finish.",
        )

    def add_base(self, amount_ml):
        self._run_exclusive(
            lambda: self._add_base_impl(amount_ml),
            "Robot is currently busy. Wait for the current action to finish.",
        )

    def _add_indicator_impl(self):
        self._ensure_ready()
        print("Adding a dew drops of indicator to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=3, which_pipette=3, amount_ml=0.15)

    def _add_acid_impl(self, amount_ml):
        amount = self._validate_volume_ml(amount_ml, reagent_name="acid")
        self._ensure_ready()
        print(f"Adding {amount} ml of acid to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=1, which_pipette=1, amount_ml=amount)

    def _add_base_impl(self, amount_ml):
        amount = self._validate_volume_ml(amount_ml, reagent_name="base")
        self._ensure_ready()
        print(f"Adding {amount} ml of base to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=2, which_pipette=2, amount_ml=amount)

    # --- Internal helpers (kept private-ish) ---
    def _ensure_ready(self):
        if not self._initialized:
            raise RuntimeError("Call initialize() before performing operations.")
        if self.simulation_mode:
            return
        if not (self.robot and self.gripper):
            raise RuntimeError("Call initialize() before performing operations.")

    def _run_exclusive(self, action, busy_message):
        with self._busy_lock:
            if self._is_busy:
                raise RuntimeError(busy_message)
            self._is_busy = True

        try:
            action()
        finally:
            with self._busy_lock:
                self._is_busy = False

    def _validate_volume_ml(self, amount_ml, reagent_name):
        try:
            amount = float(amount_ml)
        except (TypeError, ValueError):
            raise ValueError(f"Invalid {reagent_name} volume: enter a number in mL.")

        if amount <= 0:
            raise ValueError(f"Invalid {reagent_name} volume: value must be > 0 mL.")

        # Classroom-safe guardrail for accidental large inputs.
        if amount > 10:
            raise ValueError(
                f"Invalid {reagent_name} volume: value must be <= 10 mL for this demo."
            )

        return amount

    def _move_robot(self, position, speed=None, acceleration=None, blending=None):
        if self.simulation_mode:
            print(f"[SIMULATION] Move robot to position: {position}")
            return

        speed = MOVEMENT_PARAMS["speed"] if speed is None else speed
        acceleration = MOVEMENT_PARAMS["acceleration"] if acceleration is None else acceleration
        blending = MOVEMENT_PARAMS["blending"] if blending is None else blending

        self.robot.move_joint_list(
            position,
            speed,
            acceleration,
            blending,
        )

    def _operate_gripper(self, position):
        if self.simulation_mode:
            print(f"[SIMULATION] Move gripper to position: {position}")
            return
        self.gripper.move(position, 125, 125)

    def _pipetting_routine(self, input_vial, which_pipette, amount_ml):
        """Automated pipetting routine to transfer liquid from input vial to hotplate sample vial."""
        # Decode pipette choice into positions (1-4)
        if which_pipette not in [1, 2, 3, 4]:
            raise ValueError("which_pipette must be an integer between 1 and 4.")
        which_pipette = str(which_pipette)
        Above_Pipette_Holder = f"Above_Pipette_Holder_P{which_pipette}"
        Pipette_Holder = f"Pipette_Holder_P{which_pipette}"

        # Decode input vial choice into positions (1-3)
        if input_vial not in [1, 2, 3]:
            raise ValueError("input_vial must be an integer between 1 and 3.")
        input_vial = str(input_vial)

        # Higher gripper values mean more squeezing, 201 is just starting to squeeze, 241 is fully squeezed with 2ml dispensed
        if amount_ml <= 2:
            steps = int((amount_ml / 2) * 40)  # 40 steps from 201 to 241
            repeats = 1
        else:
            repeats = int(amount_ml // 2)
            remainder = amount_ml % 2
            steps = int((remainder / 2) * 40) if remainder > 0 else 0
            if remainder > 0:
                repeats += 1

        print(
            f"Pipetting {amount_ml} ml using pipette {which_pipette} from input vial {input_vial} in {repeats} full repeats and {steps} steps for remainder.",
            "\n",
        )

        # Pick up pipette
        self._move_robot(ROBOT_POSITIONS[Above_Pipette_Holder])
        print(f"Above Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)
        self._operate_gripper(180)

        self._move_robot(ROBOT_POSITIONS[Pipette_Holder])
        print(f"Picked up pipette from Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)
        self._operate_gripper(200)
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS[Above_Pipette_Holder])
        print(f"Above Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)

        for r in range(repeats):
            Above_SVH = f"Pipette_Above_SVH2_P{input_vial}"
            Just_In_SVH = f"Pipette_Just_In_SVH2_P{input_vial}"
            In_SVH = f"Pipette_In_SVH2_P{input_vial}"

            self._move_robot(ROBOT_POSITIONS[Above_SVH])
            print(f"Above Sample Vial Holder Position {input_vial}", "\n")
            time.sleep(1)

            self._move_robot(ROBOT_POSITIONS[Just_In_SVH])
            print(f"Just inside Sample Vial Holder Position {input_vial}", "\n")
            time.sleep(1)
            self._operate_gripper(240)
            time.sleep(1)
            self._move_robot(ROBOT_POSITIONS[In_SVH])
            for position in range(235, 199, -5):
                
                print("Sucking up liquid", "\n")
                self._operate_gripper(position)
                time.sleep(0.1)

            self._move_robot(ROBOT_POSITIONS[Above_SVH])
            print(f"Above Sample Vial Holder Position {input_vial}", "\n")
            time.sleep(1)

            self._move_robot(ROBOT_POSITIONS["Pipette_Near_Hotplate_Vial"])
            print("Pipette on its way to the hotplate sample vial")
            time.sleep(1)

            self._move_robot(ROBOT_POSITIONS["Pipette_Above_Hotplate_Vial"])
            print("Above sample vial on the hotplate", "\n")
            time.sleep(1)

            self._move_robot(ROBOT_POSITIONS["Pipette_In_Hotplate_Vial"])
            print("In sample vial on the hotplate", "\n")
            time.sleep(1)

            if r < repeats - 1:
                dispense_steps = 40
            else:
                dispense_steps = steps
            for position in range(201, 201 + dispense_steps, 1):
                print("Dispensing liquid", "\n")
                self._operate_gripper(position)
                time.sleep(0.1)
            
            for position in range(201 + dispense_steps - 1, 199, -5):
                print("Unsqueezing pipette", "\n")
                self._operate_gripper(position)
                time.sleep(0.1)


            self._move_robot(ROBOT_POSITIONS["Pipette_Above_Hotplate_Vial"])
            self._move_robot(ROBOT_POSITIONS["Pipette_Near_Hotplate_Vial"])
            print("Pipette on its way to the sample vial")
            time.sleep(1)

        #getting rid of excess
        Above_SVH = f"Pipette_Above_SVH2_P{input_vial}"
        Just_In_SVH = f"Pipette_Just_In_SVH2_P{input_vial}"
        In_SVH = f"Pipette_In_SVH2_P{input_vial}"

        self._move_robot(ROBOT_POSITIONS[Above_SVH])
        print(f"Above Sample Vial Holder Position {input_vial}", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS[Just_In_SVH])
        print(f"Just inside Sample Vial Holder Position {input_vial}", "\n")

        for position in range(201, 241, 1):
            print("Dispensing liquid", "\n")
            self._operate_gripper(position)
            time.sleep(0.1)
            
        for position in range(240, 199, -5):
            print("Unsqueezing pipette", "\n")
            self._operate_gripper(position)
            time.sleep(0.1)

        self._move_robot(ROBOT_POSITIONS[Above_SVH])
        print(f"Above Sample Vial Holder Position {input_vial}", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS[Above_Pipette_Holder])
        print(f"Above Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)

        self._move_robot(
            ROBOT_POSITIONS[Pipette_Holder],
            speed=0.05,
            acceleration=MOVEMENT_PARAMS["acceleration"],
            blending=MOVEMENT_PARAMS["blending"],
        )
        print(f"Replaced Pipette {which_pipette}", "\n")
        time.sleep(1)
        self._operate_gripper(180)
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS[Above_Pipette_Holder])
        print(f"Above Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS["Front_Home_Position"])
        print("Starting in home position", "\n")
        time.sleep(1)


__all__ = ["OutreachDemo"]
