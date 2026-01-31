# kn2c
# drone_imgproc
Main3.0 code runs all the prosses in parallel <br />
In main4.0 the different state of quad missions have been added.<br />
states: <br />
    STATE_DEFAULT <br />
    STATE_OBS_decrease_alt <br />
    STATE_OBS_increase_alt<br />
    STATE_QR_turnTo_dir<br />
    STATE_QR_goTo_center<br />
    STATE_QR_getOutOf_box<br />
    STATE_LINE_getBackInto_rout<br />
    STATE_LINE_landing<br />
    STATE_LINE_underAboveObs<br />
    STATE_LINE_goTo_rightAlt<br />

## optical flow
This would calculate optical flow for drone using a downward camera.<br />
The opticall flow is calculated by findding some feature points (20 or 50) in the first frame, try to find the same points in the next frame, and calculate dx and dy by camparing coresponding points.
To lessen the clculation needed, frames has been resized.

## color and obsticle avoidance
Obsticle avoidance has been done based on color area (red and yellow).

## qr code
read qr

## follow line
detect black line and follow
