% ------------------------------------------------------
% This function applies the Savitzky-Golay smoothing and derivative filter.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function filteredData = savitzkyGolayFilter(rawData, mode, windowSize, timeStep)

% Check input variables and extend raw data vector
filteredData = zeros(size(rawData));
dataLength = length(rawData);
windowSize = abs(round(windowSize));
timeStep = abs(timeStep);
if (windowSize < 5) || (windowSize > dataLength) || (mod(windowSize, 2) ~= 1)
    windowSize = 5;
    fprintf('WARNING: Window size of Savitzky-Golay filter was set to %i.\n', windowSize);
end
windowOffset = (windowSize - 1) / 2;
rawDataStart = rawData(1) - flip(rawData(2:(windowOffset + 1)) - rawData(1));
rawDataEnd = rawData(end) - flip(rawData((end - windowOffset):(end - 1)) - rawData(end));
if iscolumn(rawData)
    rawData = [rawDataStart; rawData; rawDataEnd];
else
    rawData = [rawDataStart, rawData, rawDataEnd];
end

% Compute convolution coefficients
coefficients = zeros(windowSize, 1);
for coefficientIndex = 1:windowSize
    switch mode

        case '1st derivative'
        coefficients(coefficientIndex) = ((5 * (3 * windowSize^4 - 18 * windowSize^2 + 31) * (coefficientIndex - windowOffset - 1) - 28 * (3 * windowSize^2 - 7) * (coefficientIndex - windowOffset - 1)^3) / ((windowSize * (windowSize^2 - 1) * (3 * windowSize^4 - 39 * windowSize^2 + 108)) / 15)) / timeStep;
        
        case '2nd derivative'
        coefficients(coefficientIndex) = ((12 * windowSize * (coefficientIndex - windowOffset - 1)^2 - windowSize * (windowSize^2 - 1)) / ((windowSize^2 * (windowSize^2 - 1) * (windowSize^2 - 4)) / 15)) / timeStep^2;

        case '3rd derivative'
        coefficients(coefficientIndex) = ((-(3 * windowSize^2 - 7) * (coefficientIndex - windowOffset - 1) + 20 * (coefficientIndex - windowOffset - 1)^3) / ((windowSize * (windowSize^2 - 1) * (3 * windowSize^4 - 39 * windowSize^2 + 108)) / 420)) / timeStep^3;

        otherwise
        coefficients(coefficientIndex) = ((3 * windowSize^2 - 7 - 20 * (coefficientIndex - windowOffset - 1)^2) / 4) / ((windowSize * (windowSize^2 - 4)) / 3);

    end
end

% Apply Savitzky-Golay filter according to selected mode and window size
for dataIndex = (1 + windowOffset):(dataLength + windowOffset)
    for coefficientIndex = 1:windowSize
        filteredData(dataIndex - windowOffset) = filteredData(dataIndex - windowOffset) + coefficients(coefficientIndex) * rawData(dataIndex + (coefficientIndex - windowOffset - 1));      
    end
end
