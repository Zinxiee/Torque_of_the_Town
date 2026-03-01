# Torque_of_the_Town
Mechatronics Group 1 Coursework 25WSC912

2D_similarity_transform_maths.py is for working out 2D similarity transform constants and shouldn't need to be edited or used.

detect_disc_location_from_world_frame is using the 2D similarity constants and the pixel locations of the corners of the red pickup boundry to work out where the disc is. It is uploaded to the main ESP32S3 and expects a pixel location for the centre of the disc from the camera (sent via the XIAO_CAM code). The output of this is a measurement in mm from the world frame as to the whereabouts of the white disc, however this was not outputting the correct values when I checked so will need some fixing.

XIAO_CAM is uploaded to the camera and is responsible for starting a wifi network called RobotVision, with password "TorqueOfTheTown". To see the output stream of the camera, connect to the wifi network on any device and search for http://192.168.4.1 in your browser. XIAO_CAM is responsible for image thresholding, cropping, and blob detection for the white disc. If enough white pixels are present, it finds the average x and y locations of those pixels and draws a crosshair. The centre of this crosshair (the average) is (hopefully) the centre of the disc. This central location is what is transmitted (over wire) to the ESP32S3 for use in detect_disc_location_from_world_frame.

WSC912-ESP32_motor_control is to be uploaded to the main ESP32S3 for control of the motors, currently on startup it sets the home location and then moves a certain distance. Eventually this code should be merged with the detect_disc_location_from_world_frame, as well as the Inverse Kinematics when it is written, so that its all integrated together.

For a better explanation of what each script is doing, I'd recommend pasting the code into an LLM and asking for an explanation.
