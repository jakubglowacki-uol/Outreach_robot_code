import os
import sys
import time
import math
import serial.tools.list_ports

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from utils.UR_Functions import URfunctions as URControl
from robotiq.robotiq_gripper import RobotiqGripper
from PyLabware.devices.ika_rct_digital import RCTDigitalHotplate

 
 
# === Robot Configuration ===
ROBOT_POSITIONS = {
    "Front_Home_Position": [-0.09376556078066045, -0.9369700712016602, -1.6996105909347534, -3.5858966312804164, -1.6568916479693812, 3.1620473861694336],
    "Left_Home_Position": [1.5178043842315674, -0.937005953197815, -1.6996502876281738, -3.5857583485045375, -1.6568644682513636, 3.162015914916992],

    "Pipette_Holder_P1": [-0.4267047087298792, -2.537224908868307, -1.0126903057098389, -2.732569833795065, -1.9977625052081507, 3.1414554119110107],
    "Above_Pipette_Holder_P1": [-0.4181745688067835, -2.41787113765859, -0.9318845272064209, -2.8944245777525843, -1.9620550314532679, 3.1502132415771484],

    "Pipette_Holder_P2": [-0.4771040121661585, -2.6033030949034632, -0.8818250894546509, -2.797269960443014, -2.0482032934771937, 3.1415765285491943],
    "Above_Pipette_Holder_P2": [-0.4770000616656702, -2.5076943836607875, -0.6854628920555115, -3.0892139873900355, -2.0476883093463343, 3.1417276859283447],

    "Pipette_Holder_P3": [-0.5239489714251917, -2.6828538380064906, -0.7229951620101929, -2.8765040836729945, -2.0950210730182093, 3.1417479515075684],
    "Above_Pipette_Holder_P3": [-0.5238598028766077, -2.615485807458395, -0.49711835384368896, -3.169656892816061, -2.0945642630206507, 3.1419243812561035],
    
    "Pipette_Above_SVH2_P1": [-0.5799678007708948, -1.952494283715719, -1.5227556228637695, -2.8056613407530726, -0.5792940298663538, 3.141136646270752],
    "Pipette_Just_In_SVH2_P1": [-0.5800049940692347, -2.0594665012755335, -1.6557868719100952, -2.565521379510397, -0.5797932783709925, 3.1409568786621094],
    "Pipette_In_SVH2_P1": [-0.5800517241107386, -2.164365907708639, -1.711008906364441, -2.405327459374899, -0.5801680723773401, 3.1409170627593994],
    
    "Pipette_Above_SVH2_P2": [-0.5482171217547815, -2.0075303516783656, -1.442728042602539, -2.8305393658080042, -0.5475996176349085, 3.1410045623779297],
    "Pipette_Just_In_SVH2_P2": [-0.5482919851886194, -2.1169940433897914, -1.582526683807373, -2.5811034641661585, -0.5481227079974573, 3.140824794769287],
    "Pipette_In_SVH2_P2": [-0.5482733885394495, -2.23131861309194, -1.638229250907898, -2.411039491693014, -0.5485056082354944, 3.1408092975616455],
    
    "Pipette_Above_SVH2_P3": [-0.5167034308062952, -2.0639435253539027, -1.3576841354370117, -2.8590547047057093, -0.5161336104022425, 3.1408848762512207],
    "Pipette_Just_In_SVH2_P3": [-0.5105012098895472, -2.153379579583639, -1.5025384426116943, -2.62454952816152, -0.5103495756732386, 3.1406569480895996],
    "Pipette_In_SVH2_P3": [-0.5105164686786097, -2.2685448131956996, -1.563902497291565, -2.447970529595846, -0.5107601324664515, 3.1406331062316895],

    "Pipette_Near_Hotplate_Vial": [-0.91943866411318, -1.57354797939443, -2.139638900756836, -2.526546140710348, -2.166326347981588, 3.1596767902374268],
    "Pipette_Above_Hotplate_Vial": [-0.880564037953512, -1.3417670887759705, -2.201249122619629, -2.695799490014547, -0.9592760244952601, 3.110018730163574],
    "Pipette_In_Hotplate_Vial": [-0.8806036154376429, -1.3924806875041504, -2.271242618560791, -2.5750004253783167, -0.9595149199115198, 3.109926462173462],
    
    "Pipette_Above_Pipette_Bin": [-0.6214655081378382, -1.6172100506224574, -1.9516651630401611, -2.657548566857809, -0.700897518788473, 3.0920045375823975],
    "Pipette_In_Pipette_Bin": [-0.6215084234820765, -1.934718748132223, -2.1650853157043457, -2.126490732232565, -0.7020323912249964, 3.09183669090271],


}

MOVEMENT_PARAMS = {
    "speed": 0.25,
    "acceleration": 0.5,
    "blending": 0.02,
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

    def __init__(self, robot_ip="192.168.10.2", robot_port=30003, gripper_port=63352):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.gripper_port = gripper_port
        self.robot = None
        self.gripper = None
        self.plate = None

    # --- Lifecycle ---
    def initialize(self):
        """Connect robot, gripper, hotplate; move home and start stirring."""
        if self.robot is None:
            self.robot = URControl(ip=self.robot_ip, port=self.robot_port)

        if self.gripper is None:
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

    def dispose(self):
        """Safely stop stirring/cleanup. Called automatically on destruction."""
        try:
            if self.plate:
                self.plate.stop_stirring()
                print("Stirring stopped.")
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
        self._ensure_ready()
        print("Adding a dew drops of indicator to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=3, which_pipette=3, amount_ml=0.25)

    def add_acid(self, amount_ml):
        self._ensure_ready()
        print(f"Adding {amount_ml} ml of acid to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=1, which_pipette=1, amount_ml=amount_ml)

    def add_base(self, amount_ml):
        self._ensure_ready()
        print(f"Adding {amount_ml} ml of base to the hotplate sample vial.", "\n")
        self._pipetting_routine(input_vial=2, which_pipette=2, amount_ml=amount_ml)

    # --- Internal helpers (kept private-ish) ---
    def _ensure_ready(self):
        if not (self.robot and self.gripper):
            raise RuntimeError("Call initialize() before performing operations.")

    def _move_robot(self, position):
        self.robot.move_joint_list(
            position,
            MOVEMENT_PARAMS["speed"],
            MOVEMENT_PARAMS["acceleration"],
            MOVEMENT_PARAMS["blending"],
        )

    def _operate_gripper(self, position):
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

            for position in range(235, 199, -5):
                self._move_robot(ROBOT_POSITIONS[In_SVH])
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
                self._move_robot(ROBOT_POSITIONS["Pipette_In_Hotplate_Vial"])
                print("Dispensing liquid", "\n")
                self._operate_gripper(position)
                time.sleep(0.1)

            for position in range(201 + dispense_steps - 1, 199, -5):
                self._move_robot(ROBOT_POSITIONS["Pipette_Above_Hotplate_Vial"])
                print("Unsqueezing pipette", "\n")
                self._operate_gripper(position)
                time.sleep(0.1)

            self._move_robot(ROBOT_POSITIONS["Pipette_Near_Hotplate_Vial"])
            print("Pipette on its way to the sample vial")
            time.sleep(1)

        self._move_robot(ROBOT_POSITIONS["Pipette_Above_Pipette_Bin"])
        print("Say goodbye to this pipette!", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS["Pipette_In_Pipette_Bin"])
        print("Yeet!", "\n")
        self._operate_gripper(0)
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS["Pipette_Above_Pipette_Bin"])
        print("Moved above pipette bin", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS["Front_Home_Position"])
        print("Starting in home position", "\n")
        time.sleep(1)


__all__ = ["OutreachDemo"]
