# This program prints hello world to the console every 500 milliseconds.
# Open the folder that contains this file. 
# Then click Tools > Open current folder in terminal.
# Enter the command "python3 hello.py" to execute it.
# You can use the app named Thonny to modify Python files,
# but we recommend using the above command to execute it.

import time

print("Press Ctrl+C to exit.")

while True:
	print("Hello world!")
	time.sleep(0.5)
