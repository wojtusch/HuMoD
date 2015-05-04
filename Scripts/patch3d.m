% ------------------------------------------------------
% This executes patch with remapping the coordinate system in order to
% improve navigation in plots (x -> y, y -> z, z -> x). Labels in plots
% have to be adjusted accordingly.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function h = patch3d(x, y, z, patchSpec)
h = patch(z, x, y, patchSpec);
end

