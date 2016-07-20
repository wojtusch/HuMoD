% ------------------------------------------------------
% This function rotates a vector about a given angle and axis.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function vector = rotateVector(vector, angle, axis)

if isequal(size(vector, 1), 3) && isequal(size(axis), [3, 1])
    axis = axis / norm(axis);
    R = [...
    axis(1)^2 * (1 - cos(angle)) + cos(angle), axis(1) * axis(2) * (1 - cos(angle)) - axis(3) * sin(angle), axis(1) * axis(3) * (1 - cos(angle)) + axis(2) * sin(angle); ...
    axis(2) * axis(1) * (1 - cos(angle)) + axis(3) * sin(angle), axis(2)^2 * (1 - cos(angle)) + cos(angle), axis(2) * axis(3) * (1 - cos(angle)) - axis(1) * sin(angle); ...
    axis(3) * axis(2) * (1 - cos(angle)) - axis(2) * sin(angle), axis(3) * axis(2) * (1 - cos(angle)) + axis(1) * sin(angle), axis(3)^2 * (1 - cos(angle)) + cos(angle) ...
    ];
    vector = R * vector;
else
    fprintf('ERROR: Wrong vector or axis dimension!\n');
end

end