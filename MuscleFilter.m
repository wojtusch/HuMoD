% ------------------------------------------------------
% This script processes the raw muscle data. During the measurement, the
% signals were filtered to a bandwidth between 20 Hz and 450 Hz and were
% checked for excessive amounts of line interference as well as channel
% clipping due to over-amplified signals. This script rectifies the signals
% and applies a zero-phase low-pass, moving-average or root-mean-squares
% filter with adjustable parameters. Default filter parameters were taken
% from [Konrad2005].
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Clean up workspace
clc;
clear all;
close all;

% Set parameters
filterAlgorithm = 'Root mean square';   % 'Low pass', 'Moving average' or 'Root mean square'
filterWindowSize = 100;                 % Window size in milliseconds for moving-average and root-mean-squares filter [Konrad2005]
filterHalfOrder = 3;                    % Half order of the zero-phase high-pass and low-pass filters
filterCutOff = 6;                       % Cut-off frequency of the zero-phase low-pass filter [Konrad2005]
histogramBinSteps = 16;                 % Bin steps of the histogram used for outlier removal
datasets = {
    '1.1', ...
    '1.2', ...
    '1.3', ...
    '2.1', ...
    '2.2', ...
    '2.3', ...
    '3', ...
    '4', ...
    '5.1', ...
    '5.2', ...
    '6', ...
    '7', ...
    '8' ...
};
subjects = {
    'A',...
    'B'...
};
muscleLabels = {
    'SOL_L', ...
    'SOL_R', ...
    'TIA_L', ...
    'TIA_R', ...
    'GSL_L', ...
    'GSL_R', ...
    'VSL_L', ...
    'VSL_R', ...
    'RCF_L', ...
    'RCF_R', ...
    'BCF_L', ...
    'BCF_R', ...
    'GLX_L', ...
    'GLX_R'
};

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        saveFile = getFile(subject, dataset);
        if saveFile
            [path, file, extention] = fileparts(saveFile);
            dataFile = [path, filesep, 'Raw data', filesep, file, '-RawMuscle', extention];
            variables = load(dataFile);      
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        frameRate = variables.header.Fs;
        frames = variables.header.Samples;
        muscleList = {};
        rawData = [];
        for index = 1:length(muscleLabels)
            if isfield(variables, muscleLabels{index})
                muscleList = [muscleList, muscleLabels(index)];
                rawData = [rawData; variables.(muscleLabels{index})];
            end
        end
        muscleNumber = length(muscleList);
        if (muscleNumber ~= 14) && (muscleNumber ~= 16)
            fprintf('ERROR: Not the right number of channels!\n');
            return;
        end

        % Remove outlier and rectify and filter signal
        rectifiedData = zeros(size(rawData));
        processedData = zeros(size(rawData));
        for muscleIndex = 1:muscleNumber

            % Find positve and negative peaks
            positivePeaks = [];
            negativePeaks = [];
            zeroCrossings = [1, find(diff(sign(rawData(muscleIndex, :))) ~= 0), frames];
            for intervalIndex = 1:(length(zeroCrossings) - 1)
                if median(rawData(muscleIndex, zeroCrossings(intervalIndex):zeroCrossings(intervalIndex + 1))) >= 0
                    positivePeaks = [positivePeaks, max(rawData(muscleIndex, zeroCrossings(intervalIndex):zeroCrossings(intervalIndex + 1)))];
                else
                    negativePeaks = [negativePeaks, min(rawData(muscleIndex, zeroCrossings(intervalIndex):zeroCrossings(intervalIndex + 1)))];
                end
            end
            
            % Create histograms for positive and negative peaks
            [positiveHistogram, positiveBins] = createHistogram(positivePeaks, histogramBinSteps);
            [negativeHistogram, negativeBins] = createHistogram(negativePeaks, histogramBinSteps);
            
            % Find outliers from gaps in histograms for positive and negative peaks
            gapSize = round(histogramBinSteps / 4);
            positiveGaps = strfind(positiveHistogram, zeros(1, gapSize));
            if ~isempty(positiveGaps)
                positiveOutlierLimit = positiveBins(positiveGaps(1));
                positiveOutlierIndices = find(rawData(muscleIndex, :) > positiveOutlierLimit);
            else
                positiveOutlierIndices = [];
            end
            negativeGaps = strfind(negativeHistogram, zeros(1, gapSize));
            if ~isempty(negativeGaps)
                negativeOutlierLimit = negativeBins(negativeGaps(end) + gapSize - 1);
                negativeOutlierIndices = find(rawData(muscleIndex, :) < negativeOutlierLimit);
            else
                negativeOutlierIndices = [];
            end
            
            % Remove found outliers
            if ~isempty(positiveOutlierIndices)
                rawData(muscleIndex, positiveOutlierIndices) = 0;
            end
            if ~isempty(negativeOutlierIndices)
                rawData(muscleIndex, negativeOutlierIndices) = 0;
            end
            
            % Rectify signal
            rectifiedData(muscleIndex, :) = abs(rawData(muscleIndex, :));
            
            % Apply specified filter
            switch filterAlgorithm
                
                case 'Low pass'
                % Apply zero-phase low-pass filter
                filterPasses = 2;
                filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
                [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * frameRate)));
                processedData(muscleIndex, :) = filtfilt(filterB, filterA, rectifiedData(muscleIndex, :));
                muscle.filterOrder = 2 * filterHalfOrder;
                muscle.filterCutOff = filterCutOff;
                
                case 'Moving average'
                % Apply moving-average filter
                for dataIndex = 1:frames
                    startIndex = max(1, (dataIndex - ceil((filterWindowSize / 1000 * frameRate) / 2)));
                    endIndex = min(frames, (dataIndex + ceil((filterWindowSize / 1000 * frameRate) / 2)));
                    delta = endIndex - startIndex;
                    processedData(muscleIndex, dataIndex) = sum(rectifiedData(muscleIndex, startIndex:endIndex)) / delta;
                end
                muscle.filterWindowSize = filterWindowSize;
                    
                case 'Root mean square'
                % Apply root-mean-squares filter
                for dataIndex = 1:frames
                    startIndex = max(1, (dataIndex - ceil((filterWindowSize / 1000 * frameRate) / 2)));
                    endIndex = min(frames, (dataIndex + ceil((filterWindowSize / 1000 * frameRate) / 2)));
                    delta = endIndex - startIndex;
                    processedData(muscleIndex, dataIndex) = sqrt(sum(rectifiedData(muscleIndex, startIndex:endIndex) .^2) / delta);
                end
                muscle.filterWindowSize = filterWindowSize;
                
                otherwise
                fprintf('ERROR: Invalid filter type!');
                return;
            end

        end

        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        muscle.frameRate = frameRate;
        muscle.frames = frames;
        muscle.muscleLabels = muscleList;
        muscle.filterAlgorithm = filterAlgorithm;
        muscle.filteredActivities = processedData;
        muscle = orderfields(muscle);
        if exist(saveFile, 'file') == 2
            save(saveFile, 'muscle', '-append');
        else
            save(saveFile, 'muscle');
        end

    end
end
