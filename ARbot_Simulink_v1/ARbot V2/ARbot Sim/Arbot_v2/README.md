# Arbot_v1
So far this simulation executes the trajectory that is generated at start up and joint position, velocity, acceleration, and torque are recorded using the "scope" simulink block. There are scripts that will compute the IK and trajectory (6th order) included here should you want to do more with the model. This simulation also uses the Contact Forces Library for handling the conveyor belt. 

# Folders:
#description 
Contains the urdf files for simulation and visualization (don't touch)

#Libraries 
Contact Forces Library. If Simulink complains about missing blocks or something go into this folder and look for a script called "startup_Contact_Forces.m", and run that.

#scripts data 
In this folder there is a start-up function that initializes variables used for the simulation. The simulink model will execute its initFunc which then calls "startup_ARbot.m"

#slpj
This folder is populated when Simulink runs I think. I never touch it.

