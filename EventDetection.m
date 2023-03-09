function varargout = EventDetection(varargin)
% ------------------------------------------------------
% This tool helps to detect events from the processed force data.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Begin initialization code
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @EventDetection_OpeningFcn, ...
                   'gui_OutputFcn',  @EventDetection_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end

% --- Executes just before EventDetection is made visible.
function EventDetection_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to EventDetection (see VARARGIN)

% Choose default command line output for EventDetection
handles.output = hObject;

% Set parameters
handles.initialUpperLimit = 200;        % Initial value for the adaptive upper force limit in step and jump detection
handles.minimumLowerLimit = 5;          % Minimum value for the adaptive lower force limit in step and jump detection
handles.maximumLowerLimit = 50;         % Maximum value for the adaptive lower force limit in step and jump detection
handles.histogramResolution = 200;      % Resolution of the histogram used in the upper force limit update
handles.fastStepSize = [10, 500];       % Step sizes for fast forward and reverse buttons

% Initialize variables
handles.stepMultiplier = 1;
handles.dataLoaded = 0;
handles.pan = 0;
handles.dataIndex = 1;

% Set GUI elements
set(handles.minorAxesL, 'XTick', []);
set(handles.minorAxesL, 'YTick', []);
set(handles.minorAxesM, 'XTick', []);
set(handles.minorAxesM, 'YTick', []);
set(handles.minorAxesR, 'XTick', []);
set(handles.minorAxesR, 'YTick', []);
set(handles.subjectMenu, 'String', {'A', 'B'});
set(handles.subjectMenu, 'Value', 1);
set(handles.datasetMenu, 'String', {'1.1', '1.2', '1.3', '2.1', '2.2', '2.3', '3', '4', '5.1', '5.2', '6', '7', '8'});
set(handles.datasetMenu, 'Value', 1);
set(handles.zoomInButton, 'Enable', 'off');
set(handles.zoomOutButton, 'Enable', 'off');
set(handles.panButton, 'Enable', 'off');
set(handles.previousButton, 'Enable', 'off');
set(handles.nextButton, 'Enable', 'off');
set(handles.fastForwardButton, 'Enable', 'off');
set(handles.fastReverseButton, 'Enable', 'off');
set(handles.addEvent1Button, 'Enable', 'off');
set(handles.addEvent2Button, 'Enable', 'off');
set(handles.deleteEventButton, 'Enable', 'off');
set(handles.matchForcesButton, 'Enable', 'off');
set(handles.saveButton, 'Enable', 'off');

% Add functions to search path
addpath('Scripts');

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = EventDetection_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on selection change in subjectMenu.
function subjectMenu_Callback(hObject, eventdata, handles)
% hObject    handle to subjectMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if strcmp(get(handles.datasetMenu, 'String'), 'A')
    set(handles.datasetMenu, 'String', {'1.1', '1.2', '1.3', '2.1', '2.2', '2.3', '3', '4', '5.1', '5.2', '6', '7', '8'});
    set(handles.datasetMenu, 'Value', 1);
else
    set(handles.datasetMenu, 'String', {'1.1', '1.2', '1.3', '2.1', '2.2', '2.3', '3', '4', '5.1', '5.2', '6', '7', '8'});
    set(handles.datasetMenu, 'Value', 1);
end

% --- Executes during object creation, after setting all properties.
function subjectMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to subjectMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function handles = detectEvents(handles)
% handles    structure with handles and user data (see GUIDATA)

% Detect step and jump event detection
if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
    for sideIndex = 1:2
        if sideIndex == 1
            forceY = handles.force.grfY_L;
        else
            forceY = handles.force.grfY_R;
        end
        handles.lowerLimit = max(handles.minimumLowerLimit, min(handles.maximumLowerLimit, 3 * max(forceY([1000:8000, (handles.force.frames - 7999):(handles.force.frames - 999)]))));
        handles.upperLimit = handles.initialUpperLimit;
        startIndex1 = handles.meta.startTime * handles.force.frameRate;
        endIndex1 = handles.meta.endTime * handles.force.frameRate;
        dataIndex1 = startIndex1;
        eventStart = [];
        eventEnd = [];

        % Detect initial contact event detection based on a routine written by Martin Grimmer 
        while dataIndex1 <= endIndex1
            % Find first data point that is smaller than the lower limit
            if forceY(dataIndex1) < handles.lowerLimit
                dataIndex2 = 0;
                % Find next data point that is greater than the upper limit
                while (dataIndex1 + dataIndex2 <= endIndex1) && (forceY(dataIndex1 + dataIndex2) <= handles.upperLimit(end))
                    dataIndex2 = dataIndex2 + 1;
                end
                % Find last data point that is smaller than the lower limit
                while (dataIndex1 + dataIndex2 <= endIndex1) && (forceY(dataIndex1 + dataIndex2) >= handles.lowerLimit)
                    dataIndex2 = dataIndex2 - 1;
                end
                % Go back until the first data point is greater than the last data point
                while (dataIndex1 + dataIndex2 <= endIndex1) && (dataIndex2 >= 0) && (forceY(dataIndex1 + dataIndex2 - 1) < forceY(dataIndex1 + dataIndex2))
                    dataIndex2 = dataIndex2 - 1;
                end
                % Save initial contact event
                if (dataIndex1 + dataIndex2 <= endIndex1) && (dataIndex2 >= 0)
                    eventStart = [eventStart, dataIndex1 + dataIndex2];
                end
                % Find next data point that is greater than the upper limit
                while (dataIndex1 + dataIndex2 <= endIndex1) && (forceY(dataIndex1 + dataIndex2) <= handles.upperLimit(end))
                    dataIndex2 = dataIndex2 + 1;
                end
                % Update upper limit
                if length(eventStart) >= 2
                    handles.upperLimit = [handles.upperLimit, findThreshold(forceY(eventStart(end - 1):eventStart(end)), handles.histogramResolution)];
                else
                    handles.upperLimit = [handles.upperLimit, handles.upperLimit(end)];
                end
                dataIndex1 = dataIndex1 + dataIndex2;
            else
                dataIndex1 = dataIndex1 + 1;
            end
        end
        handles.upperLimit = handles.upperLimit(1:(end - 1));

        % Detect toe off event detection based on a routine written by Martin Grimmer 
        if ~isempty(eventStart)
            eventEnd = nan(size(eventStart));
            endIndex1 = length(eventStart);
            dataIndex1 = 1;
            while dataIndex1 <= endIndex1
                startIndex2 = eventStart(dataIndex1);
                if dataIndex1 < endIndex1
                    endIndex2 = eventStart(dataIndex1 + 1);
                else
                    endIndex2 = handles.meta.endTime * handles.force.frameRate;
                end
                dataIndex2 = startIndex2;
                while dataIndex2 <= endIndex2
                    % Find first data point that is greater than the upper limit
                    if forceY(dataIndex2) > handles.upperLimit(dataIndex1)
                        dataIndex3 = 0;
                        % Find next data point that is smaller than the lower limit
                        while (dataIndex2 + dataIndex3 <= endIndex2) && (forceY(dataIndex2 + dataIndex3) >= handles.lowerLimit)
                            dataIndex3 = dataIndex3 + 1;
                        end
                        % Go forward until the first data point is greater than the last data point
                        while (dataIndex2 + dataIndex3 <= endIndex2) && (forceY(dataIndex2 + dataIndex3 + 1) < forceY(dataIndex2 + dataIndex3))
                            dataIndex3 = dataIndex3 + 1;
                        end
                        % Save toe off event
                        if dataIndex2 + dataIndex3 <= endIndex2
                            eventEnd(dataIndex1) = dataIndex2 + dataIndex3;
                        end
                        break;
                    else
                        dataIndex2 = dataIndex2 + 1;
                    end
                end
                dataIndex1 = dataIndex1 + 1;
            end
        end

        % Check for incomplete event detection
        if ~isempty(eventStart)
            for dataIndex = 1:length(eventStart)
                if isnan(eventEnd(dataIndex))
                    eventStart(dataIndex) = [];
                    eventEnd(dataIndex) = [];
                    handles.upperLimit(dataIndex) = [];
                end
            end
        end

        % Save detected event detection
        if sideIndex == 1
            handles.events.eventStart_L = eventStart;
            handles.events.eventEnd_L = eventEnd;
            handles.events.contactPhase_L = zeros(1, handles.force.frames);
            for dataIndex = 1:length(eventStart)
                handles.events.contactPhase_L(eventStart(dataIndex):eventEnd(dataIndex)) = 1;
            end
            handles.events.grfCorrection_L = zeros(1, length(eventStart));
        else
            handles.events.eventStart_R = eventStart;
            handles.events.eventEnd_R = eventEnd;
            handles.events.contactPhase_R = zeros(1, handles.force.frames);
            for dataIndex = 1:length(eventStart)
                handles.events.contactPhase_R(eventStart(dataIndex):eventEnd(dataIndex)) = 1;
            end
            handles.events.grfCorrection_R = zeros(1, length(eventStart));
        end
    end

    % Check for double event detection
    if ~strcmp(handles.dataset, '8') && ~isempty(handles.events.eventStart_L) && ~isempty(handles.events.eventStart_R)
        contactLength = median([(handles.events.eventEnd_L - handles.events.eventStart_L + 1), (handles.events.eventEnd_R - handles.events.eventStart_R + 1)]);
        durationLimit = ceil(0.6 * contactLength);
        doubleSupport = bitand(handles.events.contactPhase_L, handles.events.contactPhase_R);
        difference = diff([0, doubleSupport, 0]);
        startIndex = find(difference > 0);
        endIndex = find(difference < 0) - 1;
        eventDuration = endIndex - startIndex + 1;
        doubleEvents = (eventDuration > durationLimit);
        startIndex = startIndex(doubleEvents);
        if ~isempty(startIndex)
            for dataIndex1 = 1:length(startIndex)
                % Find the previous contact event start and decide about side
                distances_L = sort(startIndex(dataIndex1) - handles.events.eventStart_L);
                distances_R = sort(startIndex(dataIndex1) - handles.events.eventStart_R);
                distanceLimit = min([distances_L(distances_L > 0), distances_R(distances_R > 0)]);
                minimumDistance_L = distances_L(find(distances_L > distanceLimit, 1));
                minimumDistance_R = distances_R(find(distances_R > distanceLimit, 1));
                if ~isempty(minimumDistance_L) && ~isempty(minimumDistance_R) && (minimumDistance_L >= minimumDistance_R)
                    dataIndex2 = find(handles.events.eventStart_R >= startIndex(dataIndex1), 1);
                    indices = handles.events.eventStart_R(dataIndex2):handles.events.eventEnd_R(dataIndex2);
                    handles.events.contactPhase_R(indices) = 0;
                    handles.events.eventStart_R(dataIndex2) = [];
                    handles.events.eventEnd_R(dataIndex2) = [];
                    handles.events.grfCorrection_R(dataIndex2) = [];
                    handles.events.grfCorrection_L(dataIndex2) = 1;
                elseif ~isempty(minimumDistance_L) && ~isempty(minimumDistance_R)
                    dataIndex2 = find(handles.events.eventStart_L >= startIndex(dataIndex1), 1);
                    indices = handles.events.eventStart_L(dataIndex2):handles.events.eventEnd_L(dataIndex2);
                    handles.events.contactPhase_L(indices) = 0;
                    handles.events.eventStart_L(dataIndex2) = [];
                    handles.events.eventEnd_L(dataIndex2) = [];
                    handles.events.grfCorrection_L(dataIndex2) = [];
                    handles.events.grfCorrection_R(dataIndex2) = 1;
                end
            end
        end
    end
    
    % Check event detection and forces during single support phase
    if ~strcmp(handles.dataset, '8')
        dataIndex = 1;
        dataLength = length(handles.events.eventStart_L);
        while dataIndex <= dataLength
            indices = logical([zeros(1, handles.events.eventStart_L(dataIndex) - 1), bitand(handles.events.contactPhase_L(handles.events.eventStart_L(dataIndex):handles.events.eventEnd_L(dataIndex)), ~handles.events.contactPhase_R(handles.events.eventStart_L(dataIndex):handles.events.eventEnd_L(dataIndex)))]);
            if median(handles.force.grfY_R(indices)) > handles.lowerLimit
                % Find the previous contact event start and decide about side
                distances_L = sort(handles.events.eventStart_L(dataIndex) - handles.events.eventStart_L);
                distances_R = sort(handles.events.eventStart_L(dataIndex) - handles.events.eventStart_R);
                if (dataIndex <= length(handles.events.eventEnd_R)) && (dataIndex <= length(handles.events.eventEnd_L)) && any(handles.events.eventEnd_R(dataIndex) == handles.events.eventStart_L(dataIndex))
                    distanceLimit = min([distances_L(distances_L > 0), distances_R(distances_R > 0)]);
                else
                    distanceLimit = 0;
                end
                minimumDistance_L = distances_L(find(distances_L > distanceLimit, 1));
                minimumDistance_R = distances_R(find(distances_R > distanceLimit, 1));
                if ~isempty(minimumDistance_L) && ~isempty(minimumDistance_R) && (minimumDistance_L < minimumDistance_R)
                    % Shift contact event to right leg
                    handles.events.contactPhase_L(handles.events.eventStart_L(dataIndex):handles.events.eventEnd_L(dataIndex)) = 0;
                    handles.events.contactPhase_R(handles.events.eventStart_L(dataIndex):handles.events.eventEnd_L(dataIndex)) = 1;
                    handles.events.eventStart_R = sort([handles.events.eventStart_R, handles.events.eventStart_L(dataIndex)]);
                    handles.events.eventEnd_R = sort([handles.events.eventEnd_R, handles.events.eventEnd_L(dataIndex)]);
                    handles.events.eventStart_L(dataIndex) = [];
                    handles.events.eventEnd_L(dataIndex) = [];
                    handles.events.grfCorrection_R = [handles.events.grfCorrection_R(1:(dataIndex - 1)), 1 , handles.events.grfCorrection_R(dataIndex:end)];
                    handles.events.grfCorrection_L(dataIndex) = [];
                    dataLength = dataLength - 1;
                end
            end
            dataIndex = dataIndex + 1;
        end
        dataIndex = 1;
        dataLength = length(handles.events.eventStart_R);
        while dataIndex <= dataLength
            indices = logical([zeros(1, handles.events.eventStart_R(dataIndex) - 1), bitand(handles.events.contactPhase_R(handles.events.eventStart_R(dataIndex):handles.events.eventEnd_R(dataIndex)), ~handles.events.contactPhase_L(handles.events.eventStart_R(dataIndex):handles.events.eventEnd_R(dataIndex)))]);
            if median(handles.force.grfY_L(indices)) > handles.lowerLimit
                % Find the previous contact event start and decide about side
                distances_L = sort(handles.events.eventStart_R(dataIndex) - handles.events.eventStart_L);
                distances_R = sort(handles.events.eventStart_R(dataIndex) - handles.events.eventStart_R);
                if any(handles.events.eventEnd_L(dataIndex) == handles.events.eventStart_R(dataIndex))
                    distanceLimit = min([distances_L(distances_L > 0), distances_R(distances_R > 0)]);
                else
                    distanceLimit = 0;
                end
                minimumDistance_L = distances_L(find(distances_L > distanceLimit, 1));
                minimumDistance_R = distances_R(find(distances_R > distanceLimit, 1));
                if ~isempty(minimumDistance_L) && ~isempty(minimumDistance_R) && (minimumDistance_R < minimumDistance_L)
                    % Shift contact event to left leg
                    handles.events.contactPhase_R(handles.events.eventStart_R(dataIndex):handles.events.eventEnd_R(dataIndex)) = 0;
                    handles.events.contactPhase_L(handles.events.eventStart_R(dataIndex):handles.events.eventEnd_R(dataIndex)) = 1;
                    handles.events.eventStart_L = sort([handles.events.eventStart_L, handles.events.eventStart_R(dataIndex)]);
                    handles.events.eventEnd_L = sort([handles.events.eventEnd_L, handles.events.eventEnd_R(dataIndex)]);
                    handles.events.eventStart_R(dataIndex) = [];
                    handles.events.eventEnd_R(dataIndex) = [];
                    handles.events.grfCorrection_L = [handles.events.grfCorrection_L(1:(dataIndex - 1)), 1 , handles.events.grfCorrection_L(dataIndex:end)];
                    handles.events.grfCorrection_R(dataIndex) = [];
                    dataLength = dataLength - 1;
                end
            end
            dataIndex = dataIndex + 1;
        end
    end
end

function handles = plotEvents(handles)
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded   
    % Reset plots
    cla(handles.mainAxes, 'reset');
    cla(handles.minorAxesL, 'reset');
    cla(handles.minorAxesM, 'reset');
    cla(handles.minorAxesR, 'reset');
    set(handles.minorAxesL, 'XTick', []);
    set(handles.minorAxesL, 'YTick', []);
    set(handles.minorAxesL, 'ZTick', []);
    set(handles.minorAxesM, 'XTick', []);
    set(handles.minorAxesM, 'YTick', []);
    set(handles.minorAxesM, 'ZTick', []);
    set(handles.minorAxesR, 'XTick', []);
    set(handles.minorAxesR, 'YTick', []);
    set(handles.minorAxesR, 'ZTick', []);
    hold(handles.mainAxes, 'on');
    hold(handles.minorAxesL, 'on');
    hold(handles.minorAxesM, 'on');
    hold(handles.minorAxesR, 'on');
    grid(handles.mainAxes, 'on');
    axis(handles.minorAxesL, 'equal');
    axis(handles.minorAxesM, 'equal');
    axis(handles.minorAxesR, 'equal');
    xlabel(handles.mainAxes, 'Time in s');
    if ~strcmp(handles.dataset, '6')
        ylabel(handles.mainAxes, 'Force in N');
    else
        ylabel(handles.mainAxes, 'Position in mm');
    end
    
    % Plot ground reaction forces or marker motion in main axes
    if ~strcmp(handles.dataset, '6')
        time = 0:(1 / handles.force.frameRate):((handles.force.frames - 1) / handles.force.frameRate);
        plot(handles.mainAxes, time, handles.force.grfY_L, 'r');
        plot(handles.mainAxes, time, handles.force.grfY_R, 'b');
    else
        time = 0:(1 / handles.motion.frameRate):((handles.motion.frames - 1) / handles.motion.frameRate);
        plot(handles.mainAxes, time, handles.motion.markerY(14, :), 'r');
        plot(handles.mainAxes, time, handles.motion.markerY(15, :), 'b');
    end
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        eventStart = handles.eventStartList{handles.currentEvent};
        eventEnd = handles.eventEndList{handles.currentEvent};
        if strcmp(eventStart{2}, 'L') && (eventStart{3} == 0)
            eventColor = 'r';
        elseif strcmp(eventStart{2}, 'R') && (eventStart{3} == 0)
            eventColor = 'b';
        elseif strcmp(eventStart{2}, 'L')
            eventColor = [1, 0.7, 0];
        else
            eventColor = [0, 0.7, 1];
        end
        if handles.currentEvent == 1
            startTime = eventStart{1} / handles.force.frameRate;
        elseif handles.currentEvent == length(handles.eventStartList) && (handles.currentEvent >= 3)
            startTime = handles.eventStartList{handles.currentEvent - 2}{1} / handles.force.frameRate;
        elseif handles.currentEvent >= 2
            startTime = handles.eventStartList{handles.currentEvent - 1}{1} / handles.force.frameRate;
        else
            startTime = eventStart{handles.currentEvent} / handles.force.frameRate;
        end
        if handles.currentEvent == length(handles.eventStartList)
            endTime = eventEnd{1}  / handles.force.frameRate;
        elseif (handles.currentEvent == 1) && ((length(handles.eventStartList) - handles.currentEvent) >= 2)
            endTime = handles.eventEndList{handles.currentEvent + 2}{1} / handles.force.frameRate;
        elseif (length(handles.eventStartList) - handles.currentEvent) >= 1
            endTime = handles.eventEndList{handles.currentEvent + 1}{1} / handles.force.frameRate;
        else
            endTime = handles.eventEndList{handles.currentEvent}{1} / handles.force.frameRate;
        end
        xlim(handles.mainAxes, [startTime, endTime]);
    elseif strcmp(handles.dataset, '7')
        xlim(handles.mainAxes, [max(0, handles.meta.startTime + handles.currentEvent - 10), min((handles.force.frames * handles.force.frameRate), handles.meta.startTime + handles.currentEvent + 10)]);
    else
        xlim(handles.mainAxes, [max(0, handles.meta.startTime + handles.currentEvent - 10), min((handles.motion.frames * handles.motion.frameRate), handles.meta.startTime + handles.currentEvent + 10)]);
    end
    if ~strcmp(handles.dataset, '6')
        maximumValue = max(max(handles.force.grfY_L), max(handles.force.grfY_R));
        minimumValue = min(min(handles.force.grfY_L), min(handles.force.grfY_R));
    else
        maximumValue = max(max(handles.motion.markerY(14, :)), max(handles.motion.markerY(15, :)));
        minimumValue = min(min(handles.motion.markerY(14, :)), min(handles.motion.markerY(15, :)));
    end
    ylim(handles.mainAxes, [minimumValue - ((maximumValue - minimumValue) / 20), maximumValue + ((maximumValue - minimumValue) / 20)]);
    
    % Plot measurement limits in main axes
    plot(handles.mainAxes, [handles.meta.startTime, handles.meta.startTime], [minimumValue - ((maximumValue - minimumValue) / 20), maximumValue + ((maximumValue - minimumValue) / 20)], 'r--');
    plot(handles.mainAxes, [handles.meta.endTime, handles.meta.endTime], [minimumValue - ((maximumValue - minimumValue) / 20), maximumValue + ((maximumValue - minimumValue) / 20)], 'r--');
    
    % Plot time marker
    if strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        plot(handles.mainAxes, [handles.meta.startTime + handles.currentEvent, handles.meta.startTime + handles.currentEvent], [minimumValue - ((maximumValue - minimumValue) / 20), maximumValue + ((maximumValue - minimumValue) / 20)], 'r', 'LineWidth', 3);
    end
    
    % Plot color patch in main axes
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        patch([eventStart{1}, eventStart{1}, eventEnd{1}, eventEnd{1}] / handles.force.frameRate, [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'Parent', handles.mainAxes, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    elseif (strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')) && ~isempty(handles.events.eventStart) && ~isempty(handles.events.eventEnd)
        for eventIndex = 1:length(handles.events.eventStart)
            patch([handles.events.eventStart(eventIndex), handles.events.eventStart(eventIndex), handles.events.eventEnd(eventIndex), handles.events.eventEnd(eventIndex)] / handles.force.frameRate, [minimumValue, maximumValue, maximumValue, minimumValue], 'r', 'Parent', handles.mainAxes, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
        end
    end
    
    % Plot motion in left , middle and right minor axes
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        plotMotion(handles, handles.minorAxesL, round(eventStart{1} / handles.force.frameRate * handles.motion.frameRate), 90, 0);
        plotMotion(handles, handles.minorAxesM, round((eventStart{1} + (eventEnd{1} - eventStart{1}) / 2) / handles.force.frameRate * handles.motion.frameRate), 90, 0);
        plotMotion(handles, handles.minorAxesR, round(eventEnd{1} / handles.force.frameRate * handles.motion.frameRate), 90, 0);
    elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        plotMotion(handles, handles.minorAxesL, round((handles.meta.startTime + handles.currentEvent) * handles.motion.frameRate), 180, 0);
        plotMotion(handles, handles.minorAxesM, round((handles.meta.startTime + handles.currentEvent) * handles.motion.frameRate), 90, 0);
        plotMotion(handles, handles.minorAxesR, round((handles.meta.startTime + handles.currentEvent) * handles.motion.frameRate), 0, 0);
    end
    
    % Set GUI elements
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
        set(handles.eventsEdit, 'String', sprintf('%i of %i', handles.currentEvent, length(handles.eventStartList)));
    else
        set(handles.eventsEdit, 'String', sprintf('%.3f s', handles.meta.startTime + handles.currentEvent));
    end
end

function plotMotion(handles, axesHandle, index, azimuth, elevation)
% handles    structure with handles and user data (see GUIDATA)
% axesHandle handle of the target axes
% index      index of the plotted motion frame
% azimuth    viewpoint specification
% elevation  viewpoint specification

% Set current axes
if ishandle(axesHandle) && isnumeric(index) && isnumeric(azimuth) && isnumeric(elevation)
    axes(axesHandle);
    if index > 0
        % Plot joints
        plot3d(handles.motion.jointX.estimated(:, index), handles.motion.jointY.estimated(:, index), handles.motion.jointZ.estimated(:, index), 'b*');

        % Plot head
        plot3d([handles.motion.jointX.estimated(1, index), handles.motion.surfaceX(1, index), handles.motion.surfaceX(2, index), handles.motion.jointX.estimated(1, index)], [handles.motion.jointY.estimated(1, index), handles.motion.surfaceY(1, index), handles.motion.surfaceY(2, index), handles.motion.jointY.estimated(1, index)], [handles.motion.jointZ.estimated(1, index), handles.motion.surfaceZ(1, index), handles.motion.surfaceZ(2, index), handles.motion.jointZ.estimated(1, index)], 'k');

        % Plot left arm
        plot3d([handles.motion.jointX.estimated(1, index), handles.motion.jointX.estimated(2, index), handles.motion.jointX.estimated(4, index), handles.motion.markerX(8, index)], [handles.motion.jointY.estimated(1, index), handles.motion.jointY.estimated(2, index), handles.motion.jointY.estimated(4, index), handles.motion.markerY(8, index)], [handles.motion.jointZ.estimated(1, index), handles.motion.jointZ.estimated(2, index), handles.motion.jointZ.estimated(4, index), handles.motion.markerZ(8, index)], 'k');

        % Plot right arm
        plot3d([handles.motion.jointX.estimated(1, index), handles.motion.jointX.estimated(3, index), handles.motion.jointX.estimated(5, index), handles.motion.markerX(9, index)], [handles.motion.jointY.estimated(1, index), handles.motion.jointY.estimated(3, index), handles.motion.jointY.estimated(5, index), handles.motion.markerY(9, index)], [handles.motion.jointZ.estimated(1, index), handles.motion.jointZ.estimated(3, index), handles.motion.jointZ.estimated(5, index), handles.motion.markerZ(9, index)], 'k');

        % Plot spinal
        plot3d([handles.motion.jointX.estimated(7, index), handles.motion.jointX.estimated(6, index), handles.motion.jointX.estimated(1, index)], [handles.motion.jointY.estimated(7, index), handles.motion.jointY.estimated(6, index), handles.motion.jointY.estimated(1, index)], [handles.motion.jointZ.estimated(7, index), handles.motion.jointZ.estimated(6, index), handles.motion.jointZ.estimated(1, index)], 'k');

        % Plot pelvis
        plot3d([handles.motion.jointX.estimated(7, index), handles.motion.jointX.estimated(8, index), handles.motion.jointX.estimated(9, index), handles.motion.jointX.estimated(7, index)], [handles.motion.jointY.estimated(7, index), handles.motion.jointY.estimated(8, index), handles.motion.jointY.estimated(9, index), handles.motion.jointY.estimated(7, index)], [handles.motion.jointZ.estimated(7, index), handles.motion.jointZ.estimated(8, index), handles.motion.jointZ.estimated(9, index), handles.motion.jointZ.estimated(7, index)], 'k');

        % Plot left leg
        plot3d([handles.motion.jointX.estimated(14, index), handles.motion.jointX.estimated(12, index), handles.motion.jointX.estimated(10, index), handles.motion.jointX.estimated(8, index)], [handles.motion.jointY.estimated(14, index), handles.motion.jointY.estimated(12, index), handles.motion.jointY.estimated(10, index), handles.motion.jointY.estimated(8, index)], [handles.motion.jointZ.estimated(14, index), handles.motion.jointZ.estimated(12, index), handles.motion.jointZ.estimated(10, index), handles.motion.jointZ.estimated(8, index)], 'r');

        % Plot right leg
        plot3d([handles.motion.jointX.estimated(15, index), handles.motion.jointX.estimated(13, index), handles.motion.jointX.estimated(11, index), handles.motion.jointX.estimated(9, index)], [handles.motion.jointY.estimated(15, index), handles.motion.jointY.estimated(13, index), handles.motion.jointY.estimated(11, index), handles.motion.jointY.estimated(9, index)], [handles.motion.jointZ.estimated(15, index), handles.motion.jointZ.estimated(13, index), handles.motion.jointZ.estimated(11, index), handles.motion.jointZ.estimated(9, index)], 'b');

        % Plot objects
        if strcmp(handles.dataset, '5.1') || strcmp(handles.dataset, '5.2') || strcmp(handles.dataset, '7')
            plot3d([handles.motion.markerX(37, index), handles.motion.markerX(38, index), handles.motion.markerX(39, index), handles.motion.markerX(40, index)], [handles.motion.markerY(37, index), handles.motion.markerY(38, index), handles.motion.markerY(39, index), handles.motion.markerY(40, index)], [handles.motion.markerZ(37, index), handles.motion.markerZ(38, index), handles.motion.markerZ(39, index), handles.motion.markerZ(40, index)], 'k*'); 
        end
        
        % Set axes parameters
        view(azimuth, elevation);
        limits = axis;
        if length(limits) == 6
            offsetZ = 200;
            offsetX = (430 / 330 * (limits(6) - limits(5) + 2 * offsetZ) - (limits(2) - limits(1))) / 2;
            offsetY = (430 / 330 * (limits(6) - limits(5) + 2 * offsetZ) - (limits(4) - limits(3))) / 2;
            xlim([(limits(1) - offsetX), (limits(2) + offsetX)]);
            ylim([(limits(3) - offsetY), (limits(4) + offsetY)]);
            zlim([(limits(5) - offsetZ), (limits(6) + offsetZ)]);
        else
            offsetX = 200;
            xlim([(limits(1) - offsetX), (limits(2) + offsetX)]);
        end

        % Plot text
        text(0.1, 0.9, 0, sprintf('%.2f s', index / handles.motion.frameRate), 'Units', 'normalized', 'HorizontalAlignment', 'left');
        if azimuth == 90
            text(0.9, 0.9, 0, 'Right', 'Units', 'normalized', 'HorizontalAlignment', 'right');
        elseif azimuth == 180
            text(0.9, 0.9, 0, 'Front', 'Units', 'normalized', 'HorizontalAlignment', 'right');
        elseif azimuth == 0
            text(0.9, 0.9, 0, 'Back', 'Units', 'normalized', 'HorizontalAlignment', 'right');
        end
    end
end

% --- Executes on selection change in datasetMenu.
function datasetMenu_Callback(hObject, eventdata, handles)
% hObject    handle to datasetMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function datasetMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to datasetMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in loadButton.
function loadButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set GUI elements
set(handles.eventsEdit, 'String', '0 of 0');
set(handles.zoomInButton, 'Enable', 'off');
set(handles.zoomOutButton, 'Enable', 'off');
set(handles.panButton, 'Enable', 'off');
set(handles.previousButton, 'Enable', 'off');
set(handles.nextButton, 'Enable', 'off');
set(handles.fastForwardButton, 'Enable', 'off');
set(handles.fastReverseButton, 'Enable', 'off');
set(handles.addEvent1Button, 'Enable', 'off');
set(handles.addEvent2Button, 'Enable', 'off');
set(handles.deleteEventButton, 'Enable', 'off');
set(handles.matchForcesButton, 'Enable', 'off');
set(handles.saveButton, 'Enable', 'off');

% Get GUI data
datasetList = get(handles.datasetMenu, 'String');
handles.dataset = datasetList{get(handles.datasetMenu, 'Value')};
subjectList = get(handles.subjectMenu, 'String');
handles.subject = subjectList{get(handles.subjectMenu, 'Value')};

% Load data file
if isfield(handles, 'events');
    handles = rmfield(handles, 'events');
end
handles.file = getFile(handles.subject, handles.dataset);
if handles.file
    handles.variables = load(handles.file);
    if isfield(handles.variables, 'force') && isfield(handles.variables, 'meta')
        handles.force = handles.variables.force;
        handles.motion = handles.variables.motion;
        handles.meta = handles.variables.meta;
        handles.dataLoaded = 1;
        
        % Initialize variables and detect eventdetection
        if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
            handles.events.eventStart_L = [];
            handles.events.eventEnd_L = [];
            handles.events.eventStart_R = [];
            handles.events.eventEnd_R = [];
            handles = detectEvents(handles);
        elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
            handles.events.eventStart = [];
            handles.events.eventEnd = [];
            handles.events.grfCorrection = [];
        end
        
        % Create sorted event start and end lists
        if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
            handles.eventStartList = {};
            handles.eventEndList = {};
            if ~isempty(handles.events.eventStart_L) || ~isempty(handles.events.eventStart_R)
                for dataIndex = 1:length(handles.events.eventStart_L)
                        handles.eventStartList{end + 1} = {handles.events.eventStart_L(dataIndex), 'L', handles.events.grfCorrection_L(dataIndex)};
                        handles.eventEndList{end + 1} = {handles.events.eventEnd_L(dataIndex), 'L', handles.events.grfCorrection_L(dataIndex)};
                end
                for dataIndex = 1:length(handles.events.eventStart_R)
                    handles.eventStartList{end + 1} = {handles.events.eventStart_R(dataIndex), 'R', handles.events.grfCorrection_R(dataIndex)};
                    handles.eventEndList{end + 1} = {handles.events.eventEnd_R(dataIndex), 'R', handles.events.grfCorrection_R(dataIndex)};
                end
                entries = cellfun(@(x) x{1}, handles.eventStartList);
                [~, sortedIndices] = sort(entries);
                handles.eventStartList = handles.eventStartList(sortedIndices);
                handles.eventEndList = handles.eventEndList(sortedIndices);
            end
        end
        
        % Plot eventdetection
        if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
            handles.currentEvent = 1;
        else
            handles.currentEvent = 0;
        end
        handles = plotEvents(handles);

        % Set GUI elements
        set(handles.zoomInButton, 'Enable', 'on');
        set(handles.zoomOutButton, 'Enable', 'on');
        set(handles.panButton, 'Enable', 'on');
        set(handles.previousButton, 'Enable', 'on');
        set(handles.nextButton, 'Enable', 'on');
        set(handles.fastForwardButton, 'Enable', 'on');
        set(handles.fastReverseButton, 'Enable', 'on');
        if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
            set(handles.addEvent1Button, 'String', 'Add left event');
            set(handles.addEvent1Button, 'Enable', 'on');
            set(handles.addEvent2Button, 'String', 'Add right event');
            set(handles.addEvent2Button, 'Enable', 'on');
            set(handles.matchForcesButton, 'Enable', 'on');
        else
            set(handles.addEvent1Button, 'String', 'Set event start');
            set(handles.addEvent1Button, 'Enable', 'on');
            set(handles.addEvent2Button, 'String', 'Set event end');
            set(handles.addEvent2Button, 'Enable', 'off');
            set(handles.matchForcesButton, 'Enable', 'off');
        end
        set(handles.saveButton, 'Enable', 'on');
        set(handles.deleteEventButton, 'Enable', 'on');
    else
        fprintf('ERROR: No matching data found!\n');
    end
else
    fprintf('ERROR: No matching data file found!\n');
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
        % Update event and correction lists
        handles.events.eventStart_L = [];
        handles.events.eventStart_R = [];
        handles.events.grfCorrection_L = [];
        handles.events.grfCorrection_R = [];
        for eventIndex = 1:length(handles.eventStartList)
            if strcmp(handles.eventStartList{eventIndex}{2}, 'L')
                handles.events.eventStart_L = [handles.events.eventStart_L, handles.eventStartList{eventIndex}{1} / handles.force.frameRate];
                handles.events.grfCorrection_L = [handles.events.grfCorrection_L, handles.eventStartList{eventIndex}{3}];
            else
                handles.events.eventStart_R = [handles.events.eventStart_R, handles.eventStartList{eventIndex}{1} / handles.force.frameRate];
                handles.events.grfCorrection_R = [handles.events.grfCorrection_R, handles.eventStartList{eventIndex}{3}];
            end
        end
        handles.events.eventEnd_L = [];
        handles.events.eventEnd_R = [];
        for dataIndex = 1:length(handles.eventEndList)
            if strcmp(handles.eventEndList{dataIndex}{2}, 'L')
                handles.events.eventEnd_L = [handles.events.eventEnd_L, handles.eventEndList{dataIndex}{1} / handles.force.frameRate];
            else
                handles.events.eventEnd_R = [handles.events.eventEnd_R, handles.eventEndList{dataIndex}{1} / handles.force.frameRate];
            end
        end
    else
        handles.events.eventStart = handles.events.eventStart / handles.force.frameRate;
        handles.events.eventEnd = handles.events.eventEnd / handles.force.frameRate;
    end
    
    % Print number of detected events
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
        fprintf('STATUS: %i left events\n', length(handles.events.eventStart_L));
        fprintf('STATUS: %i right events\n', length(handles.events.eventStart_R));
    else
        fprintf('STATUS: %i events\n', length(handles.events.eventStart));
    end

    % Save processed data
    variables = handles.variables;
    handles.events = orderfields(handles.events);
    variables.events = handles.events;
    handles.force = orderfields(handles.force);
    variables.force = handles.force;
    save(handles.file, '-struct', 'variables');
end

% --- Executes on button press in nextButton.
function nextButton_Callback(hObject, eventdata, handles)
% hObject    handle to nextButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    % Update event pointer
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        handles.currentEvent = min(length(handles.eventStartList), handles.currentEvent + handles.stepMultiplier);
    elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        handles.currentEvent = min(handles.meta.endTime - handles.meta.startTime, handles.currentEvent + (2 * handles.stepMultiplier) / handles.motion.frameRate);
    end

    % Plot eventdetection
    handles = plotEvents(handles);

    % Update handles structure
    guidata(hObject, handles);
end


% --- Executes on button press in previousButton.
function previousButton_Callback(hObject, eventdata, handles)
% hObject    handle to previousButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    % Update event pointer
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        handles.currentEvent = max(1, handles.currentEvent - handles.stepMultiplier);
    elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        handles.currentEvent = max(0, handles.currentEvent - (2 * handles.stepMultiplier) / handles.motion.frameRate);
    end
    
    % Plot eventdetection
    handles = plotEvents(handles);

    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes on button press in fastForwardButton.
function fastForwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to fastForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    % Update event pointer
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        handles.currentEvent = min(length(handles.eventStartList), handles.currentEvent + (handles.fastStepSize(1) * handles.stepMultiplier));
    elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        handles.currentEvent = min(handles.meta.endTime - handles.meta.startTime, handles.currentEvent + (handles.fastStepSize(2) * handles.stepMultiplier) / handles.motion.frameRate);
    end
    
    % Plot eventdetection
    handles = plotEvents(handles);

    % Update handles structure
    guidata(hObject, handles);
    
end

% --- Executes on button press in fastReverseButton.
function fastReverseButton_Callback(hObject, eventdata, handles)
% hObject    handle to fastReverseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    % Update event pointer
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        handles.currentEvent = max(1, handles.currentEvent - (handles.fastStepSize(1) * handles.stepMultiplier));
    elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
        handles.currentEvent = max(0, handles.currentEvent - (handles.fastStepSize(2) * handles.stepMultiplier) / handles.motion.frameRate);
    end

    % Plot eventdetection
    handles = plotEvents(handles);

    % Update handles structure
    guidata(hObject, handles);
end

function eventsEdit_Callback(hObject, eventdata, handles)
% hObject    handle to eventsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function eventsEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to eventsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in deleteEventButton.
function deleteEventButton_Callback(hObject, eventdata, handles)
% hObject    handle to deleteEventButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    if ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7') && ~isempty(handles.eventStartList)
        % Delete current event
        if strcmp(handles.eventStartList{handles.currentEvent}{2}, 'L')
            handles.events.contactPhase_L(handles.eventStartList{handles.currentEvent}{1}:handles.eventEndList{handles.currentEvent}{1}) = 0;
        else
            handles.events.contactPhase_R(handles.eventStartList{handles.currentEvent}{1}:handles.eventEndList{handles.currentEvent}{1}) = 0;
        end
        handles.eventStartList(handles.currentEvent) = [];
        handles.eventEndList(handles.currentEvent) = [];
        handles.currentEvent = min(length(handles.eventStartList), handles.currentEvent);
    elseif (strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')) && ~isempty(handles.events.eventStart)
        % Delete last event
        handles.events.eventStart(end) = [];
        handles.events.eventEnd(end) = [];
        handles.events.grfCorrection(end) = [];
    end

    % Plot eventdetection
    handles = plotEvents(handles);

    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes on button press in addEvent1Button.
function addEvent1Button_Callback(hObject, eventdata, handles)
% hObject    handle to addEvent1Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded && ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
    % Select event start and end in main axes
    eventStartTime = [];
    eventEndTime = [];
    while 1
        [eventStartTime, ~] = ginput(1);
        if gca == handles.mainAxes
            break;
        end
    end
    while 1
        [eventEndTime, ~] = ginput(1);
        if gca == handles.mainAxes
            break;
        end
    end
    eventStartIndex = round(eventStartTime * handles.force.frameRate);
    eventEndIndex = round(eventEndTime * handles.force.frameRate);

    % Add selected event
    if ~isempty(eventStartIndex) && ~isempty(eventEndIndex) && (eventStartIndex < eventEndIndex)
        handles.eventStartList{end + 1} = {eventStartIndex, 'L', 1};
        handles.eventEndList{end + 1} = {eventEndIndex, 'L', 1};
        entries = cellfun(@(x) x{1}, handles.eventStartList);
        [~, sortedIndices] = sort(entries);
        handles.eventStartList = handles.eventStartList(sortedIndices);
        handles.eventEndList = handles.eventEndList(sortedIndices);
        handles.events.contactPhase_L(eventStartIndex:eventEndIndex) = 1;
    end 
elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
    % Add selected event start
    eventStart = round((handles.meta.startTime + handles.currentEvent) * handles.force.frameRate);
    handles.events.eventStart = [handles.events.eventStart, eventStart];
    set(handles.addEvent1Button, 'Enable', 'off');
    set(handles.addEvent2Button, 'Enable', 'on');
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in addEvent2Button.
function addEvent2Button_Callback(hObject, eventdata, handles)
% hObject    handle to addEvent2Button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded && ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
    % Select event start and end in main axes
    eventStartTime = [];
    eventEndTime = [];
    while 1
        [eventStartTime, ~] = ginput(1);
        if gca == handles.mainAxes
            break;
        end
    end
    while 1
        [eventEndTime, ~] = ginput(1);
        if gca == handles.mainAxes
            break;
        end
    end
    eventStartIndex = round(eventStartTime * handles.force.frameRate);
    eventEndIndex = round(eventEndTime * handles.force.frameRate);

    % Add selected event
    if ~isempty(eventStartIndex) && ~isempty(eventEndIndex) && (eventStartIndex < eventEndIndex)
        handles.eventStartList{end + 1} = {eventStartIndex, 'R', 1};
        handles.eventEndList{end + 1} = {eventEndIndex, 'R', 1};
        entries = cellfun(@(x) x{1}, handles.eventStartList);
        [~, sortedIndices] = sort(entries);
        handles.eventStartList = handles.eventStartList(sortedIndices);
        handles.eventEndList = handles.eventEndList(sortedIndices);
        handles.events.contactPhase_R(eventStartIndex:eventEndIndex) = 1;
    end
elseif strcmp(handles.dataset, '6') || strcmp(handles.dataset, '7')
    eventEnd = round((handles.meta.startTime + handles.currentEvent) * handles.force.frameRate);
    if eventEnd > handles.events.eventStart(end)
        % Add selected event end
        handles.events.eventEnd = [handles.events.eventEnd, eventEnd];
        handles.events.grfCorrection = [handles.events.grfCorrection, 1];
    else
        % Remove added event start
        handles.events.eventStart = handles.events.eventStart(1:(end - 1));
    end
    set(handles.addEvent1Button, 'Enable', 'on');
    set(handles.addEvent2Button, 'Enable', 'off');
end

% Plot eventdetection
handles = plotEvents(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in matchForcesButton.
function matchForcesButton_Callback(hObject, eventdata, handles)
% hObject    handle to matchForcesButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded && ~strcmp(handles.dataset, '6') && ~strcmp(handles.dataset, '7')
    % Match forces
    for eventIndex = 1:length(handles.eventStartList)
        if strcmp(handles.eventStartList{eventIndex}{2}, 'L')
            indices = logical([zeros(1, handles.eventStartList{eventIndex}{1} - 1), bitand(handles.events.contactPhase_L(handles.eventStartList{eventIndex}{1}:handles.eventEndList{eventIndex}{1}), ~handles.events.contactPhase_R(handles.eventStartList{eventIndex}{1}:handles.eventEndList{eventIndex}{1}))]);
            if (handles.eventStartList{eventIndex}{3} == 1) || (median(handles.force.grfY_R(indices)) > handles.lowerLimit)
                handles.force.grfY_L(indices) = handles.force.grfY_L(indices) + handles.force.grfY_R(indices);
                handles.force.grfY_R(indices) = 0;
            end
        elseif strcmp(handles.eventStartList{eventIndex}{2}, 'R')
            indices = logical([zeros(1, handles.eventStartList{eventIndex}{1} - 1), bitand(handles.events.contactPhase_R(handles.eventStartList{eventIndex}{1}:handles.eventEndList{eventIndex}{1}), ~handles.events.contactPhase_L(handles.eventStartList{eventIndex}{1}:handles.eventEndList{eventIndex}{1}))]);
            if (handles.eventStartList{eventIndex}{3} == 1) || (median(handles.force.grfY_L(indices)) > handles.lowerLimit)
                handles.force.grfY_R(indices) = handles.force.grfY_R(indices) + handles.force.grfY_L(indices);
                handles.force.grfY_L(indices) = 0;
            end
        end
    end
end

% Plot eventdetection
handles = plotEvents(handles);

% Update handles structure
guidata(hObject, handles);

% --- Executes on key press with focus on mainFigure or any of its controls.
function mainFigure_WindowKeyPressFcn(hObject, eventdata, handles)
% hObject    handle to mainFigure (see GCBO)
% eventdata  structure with the following fields (see FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    % Process key presses
    switch eventdata.Key
        case 'leftarrow'
            previousButton_Callback(hObject, [], handles);
        case 'rightarrow'
            nextButton_Callback(hObject, [], handles);
        case 'uparrow'
            fastForwardButton_Callback(hObject, [], handles);
        case 'downarrow'
            fastReverseButton_Callback(hObject, [], handles);
        case 's'
            % Toggle step multiplier
            if handles.stepMultiplier == 1
                handles.stepMultiplier = 5;
            else
                handles.stepMultiplier = 1;
            end
            
            % Update handles structure
            guidata(hObject, handles);
    end
      
end

% --- Executes on button press in zoomInButton.
function zoomInButton_Callback(hObject, eventdata, handles)
% hObject    handle to zoomInButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Update plot limits in main axes
axesLimits = xlim(handles.mainAxes);
startTime = axesLimits(1) + ((axesLimits(2) - axesLimits(1)) / 4);
endTime = axesLimits(2) - ((axesLimits(2) - axesLimits(1)) / 4);
xlim(handles.mainAxes, [startTime, endTime]);

% --- Executes on button press in zoomOutButton.
function zoomOutButton_Callback(hObject, eventdata, handles)
% hObject    handle to zoomOutButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Update plot limits in main axes
axesLimits = xlim(handles.mainAxes);
startTime = max(1, axesLimits(1) - ((axesLimits(2) - axesLimits(1)) / 2));
endTime = min(handles.force.frames / handles.force.frameRate, axesLimits(2) + ((axesLimits(2) - axesLimits(1)) / 2));
xlim(handles.mainAxes, [startTime, endTime]);

% --- Executes on button press in panButton.
function panButton_Callback(hObject, eventdata, handles)
% hObject    handle to panButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Toggle pan
if handles.pan == 0
    handles.pan = 1;
    pan xon;
else
    handles.pan = 0;
    pan off;
end

% Update handles structure
guidata(hObject, handles); 
