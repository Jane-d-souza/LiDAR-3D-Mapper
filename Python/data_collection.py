#Jane D'Souza
#Final Project (Deliverable 2)
#Student number: 400366436
#April 2024

#library imports for data collection
import math
import time
import serial

 #setup serial pport conenction with COM port and baud rate with 10s timeout
s = serial.Serial('COM4', 115200, timeout=10) #Port confirmed on ddevice manager as COM4 
 
print("Opening: " + s.name) #confirm port wiht print statemmnt

 #clear leftover data from serial buffers to ensure clean communciation 
s.reset_output_buffer()
s.reset_input_buffer()

 #waiting for user input (user has to press enter to start collection from TOF sensor)
input("Press Enter to start communication...")

 
myfile = open("output.xyz", "w") #open/create file to store outpu data
 
# num of measurements per layer and the toal num of layers (REMEBER: layers start at 0, L-1)
measurements_per_layer = 32
#total_layers = 10 #for hallway G (ETB)
total_layers = 2  #for demo (box)
distance_between_layers = 100 #distance between consectuvie layers 
 
for layer in range(total_layers): #loop through each layer 
    for measurement in range(measurements_per_layer): #loop through each meaurement in layer 
        data = s.readline() #read line from serial port
        value = data.decode().strip()  # strip newline charcaters from serial data

 #try to convert recvied data into an int
        try:
            distance = int(value)#convert distance value from str to int

        except ValueError: #case where conversion fails 
            print(f"Invalid data received: {value}")
            continue  # Skip to next iteration
 
        # angle calcutions (rads)
        angle_rad = 2 * math.pi * measurement / measurements_per_layer
        y = distance * math.cos(angle_rad) #y coord based on cosin of angle 
        z = distance * math.sin(angle_rad) #z coord from sin of angle 
        x = layer * distance_between_layers  # x-coord based on layer num and serpation
 
        # write coords to output file ( x y z format)
        myfile.write(f"{x} {y} {z}\n")
 
        # print vals in  console
        print(f"Layer {layer}, Measurement {measurement}: {value} -> ({x}, {y}, {z})")
 
    time.sleep(2) #wait 2 seconds before next layer (can increase or decrease based on user prefernace)
 
myfile.close() #close file after writjng all points
 
print("Closing: " + s.name)#letting me know that serial comm is being closed 
s.close() #closer serial port (COM4)
