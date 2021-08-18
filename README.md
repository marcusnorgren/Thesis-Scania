# Thesis-Scania
Code used in Thesis project: Steering Methods for Nonholonomic Motion Planning at Scania, 2021

Feel free to use and make any changes

(!) Code requires the installation of OSPQ-solver.

Steering method developed for a single TPBVP based on MIT paper.

STEERING METHOD (Several versions exist):



Steer_Dubin - uses Dubin paths - FINAL VERSION

Steer_acc - use this if acceleration is sampled (together with Polynomial_StateSpace_acc) - not used in final thesis

multiSteer - uses multi-segments & Dubins when steering - not used in final thesis

slow_mech - used in Steer_Dubin to handle low speed

PLANNER:

test_steer - main script to run the planner

Polynomial_StateSpace_NEW - custom state space in order to use the steering function - samples pos, heading, velocity

Polynomial_Validator - checks collision for the whole vehicle, somewhat simplified

CustomGoalReachedFcn - checks if goal is reached

get_Inputs_and_Plot - plots trajectory for a single steering case, and returns inputs





