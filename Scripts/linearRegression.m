% ------------------------------------------------------
% This function computes linear regression parameters.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function [regressionIntercept, regressionSlope] = linearRegression(indices, dataSegment)

if length(indices) == length(dataSegment)
    regressionSlope = (indices - mean(indices)) * (dataSegment - mean(dataSegment))' / sum((indices - mean(indices)).^2);
    regressionIntercept = mean(dataSegment) - regressionSlope * mean(indices);
else
    fprintf('ERROR: Length of indices and data segment are not equal!\n');
    regressionIntercept = [];
    regressionSlope = [];
end

end
