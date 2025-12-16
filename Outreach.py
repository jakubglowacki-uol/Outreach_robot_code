from Outreach_Core import OutreachDemo


def main():
	# Create the demo controller (IP/ports are already set for the lab setup).
	demo = OutreachDemo()

	# Connect to robot, gripper, and hotplate, move to home, and start stirring.
	demo.initialize()






	# ---------------------------------------------------------------------
	# ✏️ EDIT BELOW: choose what to add.
	# Keep the order: indicator -> acid -> base. Change the amounts if you like.
	# For safety, only adjust the numbers on the next two lines.
	acid_ml = 1   # Try values like 0.25, 0.5, 1.0 (units: mL)
	base_ml = 1    # Try values like 1.0, 2.0, 4.5 (units: mL)

	# Add a few drops of indicator.
	demo.add_indicator()
	# Add base (mL you set above).
	demo.add_base(base_ml)
	# Add acid (mL you set above).
	demo.add_acid(acid_ml)
	
	# ---------------------------------------------------------------------








	# Pause so you can observe the colour change before cleanup.
	# (Adjust if you want more/less viewing time.)
	import time
	time.sleep(5)

	# Clean up: stop stirring/plate. Also runs automatically when the program ends.
	demo.dispose()


if __name__ == "__main__":
	main()
