# Stereovision-Object-Tracking
Video presentation:
https://www.youtube.com/watch?v=7LujCM3GTeA


The app is distributed on two Raspberry Pi microcomputers with dedicated cameras connected directly via Ethernet cable.
Application works on Raspbian in version 4.14.

The system works as follows:
1. Filter out the object using HSV color space.
2. Predict next object position.
3. Define which of the objects on the screen is the tracked object.
4. Add current position to the trajectory list.
5. Draw the interface.
