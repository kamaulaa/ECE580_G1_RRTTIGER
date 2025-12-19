Hello ECE580 course staff:

This is our final project, RRT-Tiger. The structure of this repository is as follows:

Our FPGA-based RRT implementation can be found under ECE580_Final_Project.srcs, and the CPU RRT* implementation used for testing comparison is located at rrt_star.cpp.
Within ECE580_Final_Project.srcs, there are 2 subdirectories.
The first, sim_1/new, contains testbench.tv, which is the file that we used in Vivado to run simulations with our various tests and examine the waveforms.
The second subdirectory, sources_1/new, contains the actual source files that constitute our RRT-Tiger implementation.

The purpose of each source file is as follows:

core.v: Instantiates the core_ctrl and datapath modules, linking inputs/outputs between them correctly.

core_ctrl.v: Contains the controller/FSM logic.

datapath.v: Contains the datapath.

oc_array.v: The obstacle checking systolic array. Instantiated by the datapath module.

oc_pe.v: The individual PEs inside the systolic array. Instantiated by the datapath module.

quantization_block.v: Computes Manhattan distance between two points needed for cost calculation. Instantiated by the datapath module.

random_generator.v: Generates random points to be used on each iteration of the algorithm. Instantiated by the datapath module.
