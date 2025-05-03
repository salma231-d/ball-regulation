# ball-regulation with Raspberry Pi
This project is a real-time ball-balancing system using a Raspberry Pi, camera-based tracking, and PID control to stabilize a platform by controlling two servo motors

Software and Tools Used

Raspberry Pi 5 OS** (Bookworm, 64-bit)

Python 3.12+**

Autodesk Fusion 360** – CAD design for mechanical parts

OpenCV** (opencv-python==4.9.0.80) – Real-time computer vision

NumPy** (numpy==1.26.4) – Numerical operations

gpiozero** (gpiozero==1.6.2) – GPIO control of servos

picamera2** (picamera2==0.3.11) 

– Camera capture with Raspberry Pi V2 camera

-numpy>=1.24

-matplotlib>=3.7


Project Structure & Delivered Files

 branches:
 
-main-> All final documentation (delivrable,capella, Fast,block_diagrams and capella), marketing video and poster (summarizing system operation)

-code	-> Source code for the PID controller; Image-processing code for ball detection ; Integrated final code (Image processing + PID + motor control)

-test-> test of every part + simulation of the PID  (camera, motors, simulation)
