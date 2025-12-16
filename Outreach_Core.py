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
    "Front_Home_Position": [0.8972518444061279, -1.7109610042967738, 2.0420382658587855, -0.3741963666728516, 0.9695882201194763, 3.176609992980957],
    "Left_Home_Position": [0.8972518444061279, -1.7109610042967738, 2.0420382658587855, -0.3741963666728516, 0.9695882201194763, 3.176609992980957],

    "Pipette_Holder_P1": [1.0748543739318848, -1.2193072897246857, 2.045974079762594, -0.8593663734248658, 1.053755760192871, 3.1878387928009033],
    "Above_Pipette_Holder_P1": [1.075239658355713, -1.4528594625047226, 1.8489239851581019, -0.42980654657397466, 1.0532026290893555, 3.1898601055145264],

    "Pipette_Holder_P2": [0.9993770122528076, -1.1837261778167267, 2.016986672078268, -0.8671066922000428, 0.9953725337982178, 3.190044641494751],
    "Above_Pipette_Holder_P2": [0.9999032616615295, -1.4686819848469277, 1.7121833006488245, -0.2789377731135865, 0.9944725036621094, 3.192775249481201],

    "Pipette_Holder_P3": [0.9221579432487488, -1.1521427196315308, 1.9579084555255335, -0.8283539575389405, 0.9272118210792542, 3.155653476715088],
    "Above_Pipette_Holder_P3": [0.9225025773048401, -1.3648770463517685, 1.7895167509662073, -0.4483308357051392, 0.9267398715019226, 3.157646656036377],
    
    "Pipette_Above_SVH2_P1": [0.9683740139007568, -1.7790876827635707, 2.3779459635363978, -0.6283207696727295, 1.9455883502960205, 3.1621429920196533],
    "Pipette_Just_In_SVH2_P1": [0.9683740139007568, -1.7790876827635707, 2.3779459635363978, -0.6283207696727295, 1.9455883502960205, 3.1621429920196533],
    "Pipette_In_SVH2_P1": [0.9677146673202515, -1.5272654083422204, 2.510301176701681, -1.013096646671631, 1.9456119537353516, 3.160365104675293],
    
    "Pipette_Above_SVH2_P2": [0.8878871202468872, -1.8880588016905726, 2.4364917914019983, -0.5771289628795166, 1.8649230003356934, 3.164872646331787],
    "Pipette_Just_In_SVH2_P2": [0.8877210021018982, -1.8313237629332484, 2.4840994516955774, -0.6816542905620118, 1.864976167678833, 3.1644530296325684],
    "Pipette_In_SVH2_P2": [0.8871084451675415, -1.612101217309469, 2.5887988249408167, -1.005936936741211, 1.864998698234558, 3.1630024909973145],
    
    "Pipette_Above_SVH2_P3": [0.7820356488227844, -2.014886518517965, 2.515907351170675, -0.5554277461818238, 1.7579619884490967, 3.145686388015747],
    "Pipette_Just_In_SVH2_P3": [0.7818475961685181, -1.948510309258932, 2.5731874147998255, -0.6792735022357483, 1.7580888271331787, 3.145226240158081],
    "Pipette_In_SVH2_P3": [0.7809306383132935, -1.6896139583983363, 2.6902971903430384, -1.055556671028473, 1.7578468322753906, 3.14363694190979],

    "Pipette_Near_Hotplate_Vial": [1.5263832807540894, -1.7597104511656703, 1.93235952058901, -0.20739682138476567, 1.889049768447876, 3.1421141624450684],
    "Pipette_Above_Hotplate_Vial": [1.5752828121185303, -1.5506649774364014, 1.9478605429278772, -0.4315123122981568, 1.4877203702926636, 3.155264139175415],
    "Pipette_In_Hotplate_Vial": [1.5752828121185303, -1.5506649774364014, 1.9478605429278772, -0.4315123122981568, 1.4877203702926636, 3.155264139175415],
    
    "Pipette_Above_Pipette_Bin": [-0.6214655081378382, -1.6172100506224574, -1.9516651630401611, -2.657548566857809, -0.700897518788473, 3.0920045375823975],
    "Pipette_In_Pipette_Bin": [-0.6215084234820765, -1.934718748132223, -2.1650853157043457, -2.126490732232565, -0.7020323912249964, 3.09183669090271],


}

MOVEMENT_PARAMS = {
    "speed": 0.25,
    "acceleration": 0.5,
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

    def __init__(self, robot_ip="192.168.0.2", robot_port=30003, gripper_port=63352):
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



        self._move_robot(ROBOT_POSITIONS[Above_Pipette_Holder])
        print(f"Above Pipette Holder Position {which_pipette}", "\n")
        time.sleep(1)

        self._move_robot(ROBOT_POSITIONS[Pipette_Holder])
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
