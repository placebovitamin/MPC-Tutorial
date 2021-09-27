# MPC Tutorial
This tutorial (https://arxiv.org/abs/2109.11986) shows an overview of Model Predictive Control with a linear discrete-time system and constrained states and inputs. The focus is on the implementation of the method under consideration of stability and recursive feasibility. 

* Example1.m: Simple example of the regulation probelm with a discrete-time double-integrator system
* Example2.m: This example demonstrate the loss of feasiblity. 
* Example3.m: In this example, the recursive feasiblity is guaranteed due to a terminal constraint. This terminal constraints is computed at the beginning. 

In this Tutorial, the MPT3 Toolbox is used. 
(M. Herceg, M. Kvasnica, C.N. Jones, and M. Morari. Multi-Parametric Toolbox 3.0. In Proc. of the European Control Conference, pages 502–510, Zurich, Switzerland, July 17–19 2013. https://www.mpt3.org/ ) 

The plots are created with matlab2tikz (https://github.com/matlab2tikz/matlab2tikz). 
