# kn2c
drone_imgproc
Main3.0 code runs all the prosses in parallel
In main4.0 the different state of quad missions have been added.
states:
STATE_DEFAULT
STATE_OBS_decrease_alt
STATE_OBS_increase_alt
STATE_QR_turnTo_dir
STATE_QR_goTo_center
STATE_QR_getOutOf_box
STATE_LINE_getBackInto_rout
STATE_LINE_landing
STATE_LINE_underAboveObs
STATE_LINE_goTo_rightAlt

optical flow
This would calculate optical flow for drone using a downward camera.
The opticall flow is calculated by findding some feature points (20 or 50) in the first frame, try to find the same points in the next frame, and calculate dx and dy by camparing coresponding points. To lessen the clculation needed, frames has been resized.

color and obsticle avoidance
Obsticle avoidance has been done based on color area (red and yellow).

qr code
read qr

follow line
detect black line and follow

