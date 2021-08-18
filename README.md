# Thesis-Scania
Code used in Thesis project: Steering Methods for Nonholonomic Motion Planning at Scania, 2021

Feel free to use and make any changes

(!) Code requires the installation of OSPQ-solver.

Steering method developed for a single TPBVP based on MIT paper:

Richter, Charles, Adam Bry, and Nicholas Roy. “Polynomial
Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor
Environments.” Robotics Research. Ed. Masayuki Inaba and Peter
Corke. Vol. 114. Cham: Springer International Publishing, 2016. 649–
666.

STEERING METHOD (Several versions exist):



Steer_Dubin - uses Dubin paths - FINAL VERSION

Steer_acc - use this if acceleration is sampled (together with Polynomial_StateSpace_acc) - not used in final thesis

multiSteer_Dubin - uses multi-segments & Dubins when steering - not used in final thesis, but can maybe be used to complete more complicated motions - bit slower runtime

slow_mech - used in Steer_Dubin to handle low speed

PLANNER:

test_steer - main script to run the planner

Polynomial_StateSpace_NEW - custom state space in order to use the steering function - samples pos, heading, velocity

Polynomial_Validator - checks collision for the whole vehicle, somewhat simplified

CustomGoalReachedFcn - checks if goal is reached

get_Inputs_and_Plot - plots trajectory for a single steering case, and returns inputs





