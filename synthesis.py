from Synthesis_Core import SynthesisDemo


def main():
	# Create the demo controller (IP/ports are already set for the lab setup).
	demo = SynthesisDemo()

	# Connect to robot, gripper, and hotplate, move to home, and start stirring.
	demo.initialize()

	# ---------------------------------------------------------------------
	# ✏️ EDIT BELOW: choose what to add.
	# Keep the order: indicator -> acid -> base. Change the amounts if you like.
	# For safety, only adjust the numbers on the next two lines.
	cucl2_ml = 0.25   # Try values like 0.25, 0.5, 1.0 (units: mL)
	sles_ml = 4.5    # Try values like 1.0, 2.0, 4.5 (units: mL)

	# Add a few drops of indicator.
	demo.add_piroctone()
	# Add copper chloride (mL you set above).
	demo.add_cucl2(cucl2_ml)
	# Add surfactant (mL you set above).
	demo.add_sles(sles_ml)
	# ---------------------------------------------------------------------


	# Pause so you can observe the colour change before cleanup.
	# (Adjust if you want more/less viewing time.)
	import time
	time.sleep(5)

	# Clean up: stop stirring/plate. Also runs automatically when the program ends.
	demo.dispose()


if __name__ == "__main__":
	main()
