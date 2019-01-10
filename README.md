# LQR-Pitch-Control-for-Boeing-747
In this project, we have used MATLAB as well as Simulink modeling to design an autopilot that controls the pitch of an aircraft. We have designed a PID Controller and a Lead Controller for controlling the pitch of an airplane.

● We have used LQR to find the appropriate gain matrix.

● We have discretized the system and then have performed all the analysis for the same. 

● We have worked on improving some of the common drawbacks of using pre-compensators in LQR, while also pointing out another drawback of     this control technique, by using the Simulink model and during that, making our controller more robust (by testing it with the step-       disturbance)

pitch_control.m - Main code that contains all the code for pitch control.

r_scale.m - This function will find the scale factor for a full-state feedback system to eliminate the steady-state error as a pre-compensation for the LQR Control. [Can be used specifically for continuous system pnly]

pitch_control.slx - Simulink model with modelled step disturbances.
