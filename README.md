# PAM models
The pam_models package provides a ready-to-use implementation of a Hill-type muscle model with serial damping and eccentric force-velocity
relation according to publication of Häufle et al. 2014.

This package can be used for simulation and/or control of biomechanical systems as for instance the soft actuated robot developed in our Intelligent Soft Robotics Lab at Max Planck Institute for Intelligent Systems. Further information can be found [here] (https://ei.is.mpg.de/person/dbuechler).

Muscle model parameters can be provided by a parameter configuration file in JSON conform scheme. Based on these parameters along the current muscle length and the muscle activation, a realistic approximation of muscle forces can be calculated.

## Installation
For usage of the package, the whole PAM suite has to be installed according to the following [instructions] (http://people.tuebingen.mpg.de/mpi-is-software/pam/docs/).
