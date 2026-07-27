# Outreach Core API Documentation

This file explains how to use `Outreach_Core.py` as a small, configurable API for outreach and titration-style demos.

The main entry point is `OutreachDemo`. In normal use, you create the demo, initialize it, call one or more dispensing actions, then dispose it when finished.

## Quick Start

```python
from Outreach_Core import OutreachDemo

demo = OutreachDemo(simulation=True)
demo.initialize()

demo.add_indicator()
demo.add_acid(5)

demo.dispose()
```

If you are running real hardware, leave `simulation=False` and make sure the robot, gripper, and hotplate are available.

## Recommended Workflow

1. Create the controller with `OutreachDemo(...)`.
2. Call `initialize()` once before any reagent action.
3. Use `add_indicator()`, `add_acid(amount_ml)`, and `add_base(amount_ml)` as needed.
4. Call `dispose()` at the end of the program.

The class also cleans up on object deletion, but calling `dispose()` explicitly is the safest pattern.

## `OutreachDemo`

`OutreachDemo` is the user-facing wrapper in `Outreach_Core.py`. It hides the lower-level robot, gripper, and hotplate control and exposes a simple workflow.

### Constructor

```python
OutreachDemo(
    robot_ip="192.168.0.2",
    robot_port=30003,
    gripper_port=63352,
    simulation=False,
)
```

#### Parameters

- `robot_ip`: IP address of the UR robot controller.
- `robot_port`: RTDE / robot communication port.
- `gripper_port`: Robotiq gripper port.
- `simulation`: Set to `True` to skip hardware access and print simulated actions instead.

#### Common pattern

Use the defaults if your lab setup already matches them. Change them only if your robot or gripper is configured differently.

### `initialize()`

Connects the robot, gripper, and hotplate, moves the robot to the home position, and starts stirring the hotplate.

Use this before any dispensing call.

#### Notes

- In simulation mode, it only marks the demo as ready.
- In hardware mode, missing dependencies can raise an error.
- The hotplate is started at the end of initialization when it is available.

### `dispose()`

Stops the hotplate stirring and performs shutdown cleanup.

Call this when the demo is finished.

### `is_busy`

Read-only property that reports whether a dispensing action is currently running.

This is useful if you are building your own UI or automation around the class.

### `add_indicator()`

Adds the indicator reagent to the hotplate sample vial.

This method takes no arguments.

### `add_acid(amount_ml)`

Adds acid to the hotplate sample vial.

#### Parameters

- `amount_ml`: Acid volume in milliliters.

#### Accepted input

- A number such as `5` or `0.5`.
- A string that can be converted to a number, such as `"5"`.

#### Validation rules

- Must be greater than `0`.
- Must be less than or equal to `10`.
- Non-numeric input raises `ValueError`.

### `add_base(amount_ml)`

Adds base to the hotplate sample vial.

#### Parameters

- `amount_ml`: Base volume in milliliters.

#### Accepted input

- A number such as `5` or `0.5`.
- A string that can be converted to a number, such as `"5"`.

#### Validation rules

- Must be greater than `0`.
- Must be less than or equal to `10`.
- Non-numeric input raises `ValueError`.

## Example Recipes

### Example 1: Start the system and add 5 mL of acid

```python
from Outreach_Core import OutreachDemo

demo = OutreachDemo()
demo.initialize()
demo.add_acid(5)
demo.dispose()
```

### Example 2: Add indicator, then acid, then base

```python
from Outreach_Core import OutreachDemo

demo = OutreachDemo(simulation=True)
demo.initialize()
demo.add_indicator()
demo.add_acid(5)
demo.add_base(5)
demo.dispose()
```

### Example 3: Build a small user script

```python
from Outreach_Core import OutreachDemo


def run_demo(acid_ml, base_ml):
    demo = OutreachDemo()
    demo.initialize()
    demo.add_indicator()
    demo.add_base(base_ml)
    demo.add_acid(acid_ml)
    demo.dispose()


run_demo(acid_ml=1, base_ml=1)
```

## Internal Helpers

These functions are defined in `Outreach_Core.py`, but they are mainly internal helpers and usually do not need to be called from user code.

### `find_hotplate_port()`

Scans the available serial ports and tries to locate the IKA RCT hotplate.

It returns the detected port and the hotplate object when successful. If no suitable hotplate is found, it may return empty values.

### `degreestorad(angles_deg)`

Converts a list of angles from degrees to radians.

This is a simple utility helper and is not part of the typical user workflow.

### Private helper methods

The following methods are implementation details and are not intended for direct use in normal scripts:

- `_add_indicator_impl()`
- `_add_acid_impl(amount_ml)`
- `_add_base_impl(amount_ml)`
- `_ensure_ready()`
- `_run_exclusive(action, busy_message)`
- `_validate_volume_ml(amount_ml, reagent_name)`
- `_move_robot(position, speed=None, acceleration=None, blending=None)`
- `_operate_gripper(position)`
- `_pipetting_routine(input_vial, which_pipette, amount_ml)`

## Practical Notes

- The class is designed to be used as a simple lab control API, not as a fully general robot programming interface.
- `simulation=True` is the easiest way to test scripts without hardware access.
- `add_acid()` and `add_base()` accept numbers or numeric strings, which makes them easy to call from a GUI text field.
- If a dispensing command is already running, a second one will raise a busy error until the first finishes.

## Minimal Script Template

```python
from Outreach_Core import OutreachDemo


def main():
    demo = OutreachDemo(simulation=False)
    demo.initialize()

    try:
        demo.add_indicator()
        demo.add_acid(5)
    finally:
        demo.dispose()


if __name__ == "__main__":
    main()
```
