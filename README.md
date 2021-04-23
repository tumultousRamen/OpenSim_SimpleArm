# OpenSim_SimpleArm
The attached MATLAB files simulate a simple arm complete with a torso, a humerus and a radius. The biceps are attached to the two limbs and a reporters gauges and reports the fiber force experienced by the bicep upon an actuation provided by the 'brain'. The brain is a controller and actuator and a user can customize the actuation provided by simply altering the step function under the code block dedicated for the brain.
To successfully run the simulation, user must first configure their setup by running ConfigureOpenSim.m. Once executed the function provides access to the OpenSim API on MATLAB. It also provides some additional OpenSim MATLAB utility functions to the MATLAB.