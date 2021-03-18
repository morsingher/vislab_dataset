# Multi-View Clustering Algorithm

Algorithm details can be found in the pdf

TODO

- Intrinsic parameters should be read from file rather than hard-coded
- Optimize multi-threading, which is done very naively (use OpenMP?), and parallelize more modules
- Read correctly the color of points
- Integrate realtive position between multiple cameras
- Fix weird orientation on y axis and depth ranges
- Blocks can be hierarchically built in order to better distribute points and cameras
- Saving files must be done accounting for scale