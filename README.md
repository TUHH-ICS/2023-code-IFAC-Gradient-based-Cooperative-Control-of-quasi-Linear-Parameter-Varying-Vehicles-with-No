# Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients

## General

This repository contains the simulation code to reproduce the tables and figures presented in

> Gradient-based Cooperative Control ofquasi-Linear Parameter Varying Vehicleswith Noisy Gradients

The code has the following entry point:
1. Figures for Section 5.1 "Single LTI Quadrotor" (corresponding to Fig.2-3 from the paper): Main_quadrotor_example.m 
2. Figures for Section 5.2 "Multiple Interacting Vehicles with Non-Linear Friction" (corresponding to Fig.5-8 from the paper): Main.m 


These will produce among others the material presented in the paper. 

## Prerequisites

To run Main_quadrotor_example.m, following toolboxes need to be installed:
1. [CVX](http://cvxr.com/cvx/download/)

To run Main.m, some additional toolboxes need to be installed:
1. [YALMIP](https://yalmip.github.io/download/)
2. [sdpt3](https://github.com/SQLP/SDPT3)


The simulation code in this repository was tested in the following environment:
* *Windows 10 Pro* Version 21H2
* *Matlab* 2021b
