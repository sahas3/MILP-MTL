# MILP-MTL
An MILP Approach for Real-time Optimal Controller Synthesis with Metric Temporal Logic Specifications
See https://arxiv.org/pdf/1603.02650v1.pdf for details.

# Installing Dependencies

This code depends on YALMIP, which can be obtained with the Multi-Parametric Toolbox, or MPT3, 
see http://control.ee.ethz.ch/~mpt/3/Main/Installation. MPT is also required for plotting polyhedras.

Computing robustness for the MTL specifications depends on the s-TaLiRo tool, which can be downloaded from
https://sites.google.com/a/asu.edu/s-taliro/s-taliro/download

We use the Gurobi solver as back-end to solve the optimization problem, though other solvers might work as well. 
For the user-interactive example to work without modifications, Gurobi needs to be installed and configured for Matlab. 
See http://www.gurobi.com.

The user-interactive example also uses a customized ginput.m file by Jiro Doke (http://www.mathworks.com/matlabcentral/fileexchange/38703-custom-ginput/content/ginputc.m) and is included here.

# Example

Once everything is installed, you can run the file 'find_OptimalTraj_mpc_m3pi.m' and choose either to run the in-built 
example or provide a reach-avoid MTL specification of your choice.

# Contact Us

You can contact sayan.jubiee@gmail.com for any queries or to report any bugs in the code.
