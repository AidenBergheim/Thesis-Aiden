**Background**

The files in this repository act as supporting materials for the thesis paper titled 'Predefined-Time Localization and Circumnavigation of 
Multiple Targets using Bearing-Only Measurements' by Aiden Bergheim. This paper considers the problem of controlling an autonomous agent to 
simultaneously localize and circumnavigate multiple unknown stationary targets using bearing-only measurements and the agent’s known position. 

**Structure**

The contents of the repository are summarised as follows:

1. _Experimental Files_: files relating to the experimental validation of the proposed algorithms for a holonomic agent, performed using a Crazyflie 2.1 drone.
    - _Python Algorithm Adaptation_: Contains the python adaptation of the proposed algorithms for a holonomic agent used for experimental testing, based of those produced by Donglin Sui (https://github.com/Gloogger).
    - _Results_: Contains the .csv files from experimental testing as well as MATLAB scripts to plot this data.
2. Simulation Files: files used to perform simulation of algorithms proposed for each holonomic and non-holomic agent models.
    - _MATLAB Code_: Contains simulation scripts using Euler integration of the proposed algorithms and those of other researchers.
    - _Results_: Contains the results of executing the MATLAB code in .fig and .svg formats.

**Simulations**

Each simulation result presented in the thesis paper has an associated file to run it. For example, the simulation of the proposed algorithms for a holonomic agent
when noise is applied to bearing measurements is titled 'Holonomic_Noise.m'. Running this file runs the relevant simulation, and plots are automatically produced upon
completion. Agent files are created for each set of proposed algorithms, for example for the proposed algorithms for a non-holonomic agent, the agent is called 'Agent_Nonholonomic.m'. These agent files contain the relevant algorithms and are called by the relevant main files.

**Videos**

In addition to the provided files, videos of the experimental validation performed can be found at: https://drive.google.com/drive/folders/1Ub0Wb97BTyFVtbwuX7Sc5nAbCrVoyXc6?usp=drive_link.
