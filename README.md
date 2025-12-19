# Epson SCARA Palletizing Project
## CS5553 Fall 2025
## Lacey Bowden Final Graduate Project

main.py runs the main Python code. This pulls from two auxillary files: modbus_fxns.py and camera_fxns.py.
epson_code has the SPEL code for the SCARA robot side. 
Modbus TCP is used to communicate between the two systems to palletize "good" (white-capped) bottles and "bad" (orange-capped bottles).
