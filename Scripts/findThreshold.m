% ------------------------------------------------------
% This function implements a method for clustering-based thresholding
% according to [Otsu1979].
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function [threshold, bins, histogram] = findThreshold(data, binSteps)

% Create histogram
[histogram, bins] = createHistogram(data, binSteps);
binDifference = abs(bins(2) - bins(1));

% Compute threshold
sumAB = 0;
sumB = 0;
weightB = 0;
maxVariance = 0;
threshold = 0;
for binIndex = 1:binSteps
    sumAB = sumAB + (bins(binIndex) + binDifference / 2) * histogram(binIndex);
end
for binIndex = 1:binSteps
    weightB = weightB + histogram(binIndex);
    if weightB == 0
        continue;
    end
    weightA = length(data) - weightB;
    if weightA == 0
        break;
    end
    sumB = sumB + (bins(binIndex) + binDifference / 2) * histogram(binIndex);
    meanB = sumB / weightB;
    meanA = (sumAB - sumB) / weightA;
    betweenVariance = weightB * weightA * (meanB - meanA)^2;
    if betweenVariance > maxVariance
      maxVariance = betweenVariance;
      threshold = (bins(binIndex) + binDifference / 2);
    end
end

end