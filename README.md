# UCI EASEL State Estimation Work
Currently exploring and benchmarking common state estimators for the spacecraft rendezvous/docking problem.

## Problem Formulation
Dynamics of the system are for a target-centered coordinate system, where the origin represents the target spacecraft. 
The goal of the problem is to rendezvous to the target spacecraft given a relative trajectory away from it and dock under the target spacecraft.

## Simulating nonlinear orbital dynamics.
The simulation of this system assumes that the spacecraft follows the nonlinear ODE's specified in the "Chance-constrained model predictive control for spacecraft rendezvous with disturbance estimation" paper. It takes the ODE's formulated, and numerically solves the next steps of the spacecraft numerically using MATLAB's ODE solver.

## State estimators and optimal controllers
The optimal controllers used in this repository all assume that the chaser spacecraft will follow the Hill-Clohessy-Wiltshire equations that approximately linearizes the above nonlinear dynamics.

