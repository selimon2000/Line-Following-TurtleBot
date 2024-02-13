# Line-Following Turtlebot with Glass Detection

## Components

### Line Following Package
The line following package is a ROS package to enable the Turtlebot to autonomously follow a line path. It does so using a USB Camera, where it identifies the midpoint of a yellow line, so that it continuously tries to follow it such that the blob is at the centre of the image. It does so by blurring, converting to HSV, and thresholding the image to follow the line of known colour, before finding the binary blob, using OpenCV.

### Arduino Code
The Arduino code is connected with an ultrasonic sensor. This sensor plays a crucial role in detecting obstacles, specifically glass surfaces, to prevent collisions, as with vision glass obstacles, such as glass doors, cannot be detected. The Arduino Uno is connected to the USB port of the Raspberry Pi of the Turtlebot 3, which is connected via Rosserial.

## Video Demonstration

For a visual demonstration of the line-following Turtlebot with glass detection, refer to the following video:

[![Line-Following Turtlebot with Glass Detection](https://img.youtube.com/vi/w0Koadnqfbs/0.jpg)](https://youtu.be/w0Koadnqfbs)

Click the image above or visit [this link](https://youtu.be/w0Koadnqfbs) to watch the video.
