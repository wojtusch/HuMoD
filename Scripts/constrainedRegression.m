% ------------------------------------------------------
% This function solves a constrained least-squares problem for a given data
% segment.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function dataSegment = constrainedRegression(dataSegment, order)

if length(dataSegment) < 6
    fprintf('ERROR: Data segment does not have enough data points!\n');
    return;
elseif order < 3
    fprintf('ERROR: Order needs to be greater than three!\n');
    return;
elseif isnan(dataSegment(1)) || isnan(dataSegment(2)) || isnan(dataSegment(3)) || isnan(dataSegment(end - 2))  || isnan(dataSegment(end - 1))  || isnan(dataSegment(end))
    fprintf('ERROR: First or last three data points are NaN!\n');
    return;
end
datax = find(~isnan(dataSegment));
datay = dataSegment(~isnan(dataSegment));
datady = zeros(2, 1);
datady(1) = (-datay(3) + 4 * datay(2) - 3 * datay(1)) / 2;
datady(2) = (3 * datay(end) - 4 * datay(end - 1) + datay(end - 2)) / 2;
dataX = 1:length(dataSegment);
order = min(order, (length(datax) - 1));
datax = datax(:);
d = datay(:);
C(:, (order + 1)) = ones(length(datax), 1);
for orderIndex = order:(-1):1
    C(:, orderIndex) = datax .* C(:, (orderIndex + 1));
end

% Define equality constraints for end points and derivatives
A = [ ...
    datax(1) .^ (order:(-1):0); ...
    [((order:(-1):1) .* datax(1) .^ ((order - 1):(-1):0)), 0]; ...
    datax(end) .^ (order:(-1):0); ...
    [((order:(-1):1) .* datax(end) .^ ((order - 1):(-1):0)), 0] ...
    ];
b = [ ...
    datay(1); ...
    datady(1); ...
    datay(end); ...
    datady(2) ...
    ];

% Solve constrained linear least-squares problem 
options = optimset('LargeScale', 'off', 'Display', 'off');
p = lsqlin(C, d, [], [], A, b, [], [], [], options);
dataSegment = polyval(p, dataX);