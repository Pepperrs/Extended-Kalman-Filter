# Extended Kalman Filter Project 

This project builds a Kalman filter ontop of udacities framework and many of their code examples.

## Steps of the Kalman Filter

1. Initialize the filter with the first measurement taken.
2. after the first measurements have been taken, predict the current location of the pedestrian 
according to previous movement and position in relation to the time passed.
3. Update the state of the pedestrian by new data gained by either LiDAR or RADAR.

    1. Laser - set up the LiDAR matricies, then update the state of the pedestrian 
    according to sensors and current state 
    2. Radar - set up the RADAR matricies, then update the state of the pedestrian 
    according to sensors and current state

4. go to step 2.
