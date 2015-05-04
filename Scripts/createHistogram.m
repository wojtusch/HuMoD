% ------------------------------------------------------
% This function creats a histogram from the give data and bin parameters.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function [histogram, bins] = createHistogram(data, binSteps)

minValue = min(data);
maxValue = max(data);
bins = linspace(minValue, maxValue, binSteps + 1);
histogram = zeros(size(bins));
for dataIndex = 1:length(data)
    binIndex = find(bins <= data(dataIndex), 1, 'last');
    histogram(binIndex) = histogram(binIndex) + 1;
end
histogram(end - 1) = histogram(end - 1) + histogram(end);
histogram = histogram(1:end - 1);
bins = bins(1:end - 1);

end