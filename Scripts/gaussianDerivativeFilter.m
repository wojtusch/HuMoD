% ------------------------------------------------------
% This function applies the Gaussian derivative filter.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function filteredData = gaussianDerivativeFilter(rawData, sigma)

% Compute Gaussian and Gaussian derivative
x = floor(-3 * sigma):ceil(3 * sigma);
G = exp(-0.5 * x.^2 / sigma^2);
G = G / sum(G);
dG = -x .* G / sigma^2;
filteredData = conv(rawData, dG);

% Crop bounds
filterOffset = (length(dG) - 1) / 2;
dataLength = length(filteredData);
filteredData = filteredData((filterOffset + 1):(dataLength - filterOffset));

