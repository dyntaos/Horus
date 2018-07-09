# Horus
Horus - Arduino model rocket recovery and data recording system

This code was created to use a GY-87 10DOF sensor module and is currently being tested and developed on an Arduino Uno R3,
 to be later adapted for the Arduino Pro Mini for "launch" mode.


#The intended purpose of this system is to perform 2 main functions:
    -Record flight data:
        -Maximum altitude
        -Time to apogee, parachute deployment, touchdown
    -Delay the deployment of the rocket until a lower altitude, to prevent winds from carrying the rocket excessively far
        from the launch site. At apogee, the engines recovery charge will initiate a separation of the body tubes (tumble
        recovery), then later the parachute will be deployed at a preset altitude, performed by the Arduino via MOSFET 
        triggered black-powder charge.

#Why?
This rocket I am designing this for is projected to fly just under 1km in altitude and if its parachute deploys at 1km, with
even light winds, I will likely never recover the rocket, let alone see it again.


#Currently this project is a work in progress and is not fully operational yet!

More documentation will be written when the code is fully operational.

Contact me if you want further information on the project
