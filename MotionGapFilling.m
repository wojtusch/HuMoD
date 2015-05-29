function varargout = MotionGapFilling(varargin)

% ------------------------------------------------------
% This tool helps to fill gaps in motion data and transforms the
% reference frame according to [Wu1995].
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Begin initialization code
gui_Singleton = 1;
gui_State = struct('gui_Name', mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MotionGapFilling_OpeningFcn, ...
    'gui_OutputFcn',  @MotionGapFilling_OutputFcn, ...
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

% --- Executes just before mainFigure is made visible.
function MotionGapFilling_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mainFigure (see VARARGIN)

% Clear text output
clc;

% Choose default command line output for mainFigure
handles.output = hObject;

% Set parameters
handles.dataLoaded = 0;

% Set GUI elements
set(handles.dataFileEdit, 'String', pwd);
set(handles.variableEdit, 'String', 'Motion');
set(handles.frameEdit, 'String', '0.5');
set(handles.leftSegmentEdit, 'String', '30');
set(handles.rightSegmentEdit, 'String', '30');
set(handles.gapsTable, 'Enable', 'off');
set(handles.gapsTable, 'ColumnName', []);
set(handles.gapsTable, 'Data', []);
set(handles.infoLabel, 'String', 'No data');
set(handles.methodMenu, 'String', {'constrained fit', 'unconstrained fit', 'linear', 'nearest', 'pchip', 'spline'});
set(handles.methodMenu, 'Value', 1);
set(handles.orderEdit, 'string', '7');
handles.leftTrajectories = [1, 4, 6, 8, 14, 16, 19, 21, 23, 21, 23, 25, 27, 29, 31, 33, 35];
handles.rightTrajectories = [2, 5, 7, 9, 15, 17, 20, 22, 24, 22, 24, 26, 28, 30, 32, 34, 36];
handles.trajectoryIndex = nan;
handles.gapIndex = nan;

% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = MotionGapFilling_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function handles = checkData(handles)
% handles    structure with handles and user data (see GUIDATA)

% Clear variables
if isfield(handles, 'gaps')
    handles = rmfield(handles, 'gaps');
end
if isfield(handles, 'dimensions')
    handles = rmfield(handles, 'dimensions');
end
set(handles.gapsTable, 'ColumnName', []);
set(handles.gapsTable, 'Data', []);
set(handles.gapsTable, 'Enable', 'off');

% Process data file
matData = load(get(handles.dataFileEdit, 'String'));
variableNames = fieldnames(matData);
variableIndex = ~cellfun(@isempty, regexp(variableNames, get(handles.variableEdit, 'String')));
handles.variableName = variableNames(variableIndex);
if ~isempty(handles.variableName)
    % Read data and meta information
    handles.frameRate = matData.(handles.variableName{1}).FrameRate;
    handles.frames = matData.(handles.variableName{1}).Frames;
    handles.labels = matData.(handles.variableName{1}).Trajectories.Labeled.Labels;
    handles.data = matData.(handles.variableName{1}).Trajectories.Labeled.Data;
    
    % Count and measure gaps
    handles.dimensions = size(handles.data);
    if handles.frames ~= handles.dimensions(3)
        msgbox('Number of frames does not match!', 'ERROR', 'error');
        return;
    end
    handles.gaps(1:handles.dimensions(1)) = struct('number', 0, 'startIndex', [], 'endIndex', [], 'formatString', '');
    for trajectoryIndex = 1:handles.dimensions(1)
        nanFlag = 0;
        for dataIndex = 1:handles.dimensions(3)
            if isnan(handles.data(trajectoryIndex, 1, dataIndex)) && (nanFlag == 0)
                handles.gaps(trajectoryIndex).number = handles.gaps(trajectoryIndex).number + 1;
                handles.gaps(trajectoryIndex).startIndex = [handles.gaps(trajectoryIndex).startIndex, dataIndex];
                nanFlag = 1;
            elseif ~isnan(handles.data(trajectoryIndex, 1, dataIndex)) && (nanFlag == 1)
                handles.gaps(trajectoryIndex).endIndex = [handles.gaps(trajectoryIndex).endIndex, (dataIndex - 1)];
                nanFlag = 0;
            end
        end
        if handles.gaps(trajectoryIndex).number > 0
            handles.gaps(trajectoryIndex).formatString = ['<html><span style="color:#AA0000; font-weight:bold;">', num2str(handles.gaps(trajectoryIndex).number), '</span></html>'];
        else
            handles.gaps(trajectoryIndex).formatString = ['<html><span style="color:#00AA00;">', num2str(handles.gaps(trajectoryIndex).number), '</span></html>'];
        end
        if length(handles.gaps(trajectoryIndex).startIndex) ~= length(handles.gaps(trajectoryIndex).endIndex)
            handles.gaps(trajectoryIndex).endIndex = [handles.gaps(trajectoryIndex).endIndex, handles.frames];
        end
        if length(handles.gaps(trajectoryIndex).startIndex) ~= length(handles.gaps(trajectoryIndex).endIndex)
            msgbox('Indices of recognized gaps do not fit!', 'ERROR', 'error');
            return;
        end
    end
    
    % Show gaps in GUI
    set(handles.gapsTable, 'ColumnName', handles.labels);
    set(handles.gapsTable, 'Data', {handles.gaps(:).formatString});
    set(handles.gapsTable, 'Enable', 'on');
    
    % Set data loaded flag
    handles.dataLoaded = 1;
else
    msgbox('Given variable was not found!', 'ERROR', 'error');
    return;
end

function handles = plotGap(handles)
% hObject    handle to frameEdit (see GCBO)

% Clear plot
cla(handles.xAxis);
cla(handles.yAxis);
cla(handles.zAxis);
cla(handles.eAxis);

% Check for a mirror trajectory
if get(handles.mirrorCheckbox, 'Value') && (~isempty(find(handles.leftTrajectories == handles.trajectoryIndex, 1)))
    handles.mirrorIndex = handles.trajectoryIndex + 1;
elseif get(handles.mirrorCheckbox, 'Value') && (~isempty(find(handles.rightTrajectories == handles.trajectoryIndex, 1)))
    handles.mirrorIndex = handles.trajectoryIndex - 1;
else
    handles.mirrorIndex = 0;
end

% Compute trajectory data and plot parameters
timeFrame = str2double(get(handles.frameEdit, 'String'));
set(handles.infoLabel, 'String', [num2str(handles.gapIndex), ' of ', num2str(handles.gaps(handles.trajectoryIndex).number), ' in ', handles.labels{handles.trajectoryIndex}]);
tData = 0:(1 / handles.frameRate):(handles.frames - 1) / handles.frameRate;
xData = squeeze(handles.data(handles.trajectoryIndex, 1, :));
yData = squeeze(handles.data(handles.trajectoryIndex, 2, :));
zData = squeeze(handles.data(handles.trajectoryIndex, 3, :));
eData = squeeze(handles.data(handles.trajectoryIndex, 4, :));
if (handles.mirrorIndex ~= 0)
    xMirror = squeeze(handles.data(handles.mirrorIndex, 1, :));
    yMirror = squeeze(handles.data(handles.mirrorIndex, 2, :));
    zMirror = squeeze(handles.data(handles.mirrorIndex, 3, :));
    eMirror = squeeze(handles.data(handles.mirrorIndex, 4, :));
    xMirror = xMirror + (nanmean(xData) - nanmean(xMirror));
    yMirror = yMirror + (nanmean(yData) - nanmean(yMirror));
    zMirror = zMirror + (nanmean(zData) - nanmean(zMirror));
    eMirror = eMirror + (nanmean(eData) - nanmean(eMirror));
end
startTime = max((((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1) / handles.frameRate) - timeFrame), 0);
endTime = min((((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - 1) / handles.frameRate) + timeFrame), ((handles.frames - 1) / handles.frameRate));

% Plot all trajectories around the current gap
plot(handles.xAxis, tData, xData, 'k', 'LineWidth', 2);
xlim(handles.xAxis, [startTime, endTime]);
ylabel(handles.xAxis, 'x');
grid(handles.xAxis, 'on');
hold(handles.xAxis, 'on');
set(handles.xAxis, 'XTickLabel', []);
plot(handles.yAxis, tData, yData, 'k', 'LineWidth', 2);
xlim(handles.yAxis, [startTime, endTime]);
ylabel(handles.yAxis, 'y');
grid(handles.yAxis, 'on');
hold(handles.yAxis, 'on');
set(handles.yAxis, 'XTickLabel', []);
plot(handles.zAxis, tData, zData, 'k', 'LineWidth', 2);
xlim(handles.zAxis, [startTime, endTime]);
ylabel(handles.zAxis, 'z');
grid(handles.zAxis, 'on');
hold(handles.zAxis, 'on');
set(handles.zAxis, 'XTickLabel', []);
plot(handles.eAxis, tData, eData, 'k', 'LineWidth', 2);
xlim(handles.eAxis, [startTime, endTime]);
xlabel(handles.eAxis, 't')
ylabel(handles.eAxis, 'e');
grid(handles.eAxis, 'on');
hold(handles.eAxis, 'on');

% Plot mirror trajectory around the current gap
if (handles.mirrorIndex ~= 0)
    plot(handles.xAxis, tData, xMirror, 'b--');
    plot(handles.yAxis, tData, yMirror, 'b--');
    plot(handles.zAxis, tData, zMirror, 'b--');
    plot(handles.eAxis, tData, eMirror, 'b--');
end

% Compute and plot interpolant
leftDataSegment = max((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - round(str2double(get(handles.leftSegmentEdit, 'String')))), 1);
rightDataSegment = min((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) + round(str2double(get(handles.rightSegmentEdit, 'String')))), handles.frames);
methodList = get(handles.methodMenu, 'String');
method = methodList{get(handles.methodMenu, 'Value')};
if (abs(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - leftDataSegment) < 3)
    answer = questdlg('Do you want to copy the first valid value to fill the gap?', 'Not enough data points on the left side of the gap', 'Yes', 'No', 'No');
    if strcmp(answer, 'Yes')
        handles.xApproximation = nan(size(xData));
        handles.xApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = xData(handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) + 1);
        handles.yApproximation = nan(size(xData));
        handles.yApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = yData(handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) + 1);
        handles.zApproximation = nan(size(xData));
        handles.zApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = zData(handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) + 1);
        plot(handles.xAxis, tData, handles.xApproximation, 'r--');
        plot(handles.yAxis, tData, handles.yApproximation, 'r--');
        plot(handles.zAxis, tData, handles.zApproximation, 'r--');
    end
    set(handles.infoLabel, 'String', 'No Data');
elseif (abs(handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - rightDataSegment) < 3)
    answer = questdlg('Do you want to copy the last valid value to fill the gap?', 'Not enough data points on the right side of the gap', 'Yes', 'No', 'No');
    if strcmp(answer, 'Yes')
        handles.xApproximation = nan(size(xData));
        handles.xApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = xData(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1);
        handles.yApproximation = nan(size(xData));
        handles.yApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = yData(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1);
        handles.zApproximation = nan(size(xData));
        handles.zApproximation(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex):handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex)) = zData(handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1);
        plot(handles.xAxis, tData, handles.xApproximation, 'r--');
        plot(handles.yAxis, tData, handles.yApproximation, 'r--');
        plot(handles.zAxis, tData, handles.zApproximation, 'r--');
    end
    set(handles.infoLabel, 'String', 'No Data');
else
    order = round(str2double(get(handles.orderEdit, 'String')));
    handles.xApproximation = nan(size(xData));
    [handles.xApproximation(leftDataSegment:rightDataSegment), handles.xConstraints, handles.xError] = approximate(xData(leftDataSegment:rightDataSegment), method, order);
    handles.yApproximation = nan(size(xData));
    [handles.yApproximation(leftDataSegment:rightDataSegment), handles.yConstraints, handles.yError] = approximate(yData(leftDataSegment:rightDataSegment), method, order);
    handles.zApproximation = nan(size(xData));
    [handles.zApproximation(leftDataSegment:rightDataSegment), handles.zConstraints, handles.zError] = approximate(zData(leftDataSegment:rightDataSegment), method, order);
    plot(handles.xAxis, tData, handles.xApproximation, 'r--');
    plot(handles.yAxis, tData, handles.yApproximation, 'r--');
    plot(handles.zAxis, tData, handles.zApproximation, 'r--');
    handles.constraints = handles.xConstraints | handles.yConstraints | handles.zConstraints;
    if handles.constraints(1)
        set(handles.leftConstraintsEdit, 'BackgroundColor', [0.5, 0, 0]);
    else
        set(handles.leftConstraintsEdit, 'BackgroundColor', [0, 0.5, 0]);
    end
    if handles.constraints(2)
        set(handles.rightConstraintsEdit, 'BackgroundColor', [0.5, 0, 0]);
    else
        set(handles.rightConstraintsEdit, 'BackgroundColor', [0, 0.5, 0]);
    end
    handles.error = handles.xError + handles.yError + handles.zError;
    set(handles.errorEdit, 'String', num2str(handles.error, 6));
end

% Plot left and right data segment limits
leftTimeSegment = round(str2double(get(handles.leftSegmentEdit, 'String'))) / handles.frameRate;
rightTimeSegment = round(str2double(get(handles.rightSegmentEdit, 'String'))) / handles.frameRate;
plot(handles.xAxis, max((((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1) / handles.frameRate) - leftTimeSegment), 0) * ones(1, 2), ylim(handles.xAxis), 'b');
plot(handles.xAxis, min((((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - 1) / handles.frameRate) + rightTimeSegment), ((handles.frames - 1) / handles.frameRate)) * ones(1, 2), ylim(handles.xAxis), 'b');
plot(handles.yAxis, max((((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1) / handles.frameRate) - leftTimeSegment), 0) * ones(1, 2), ylim(handles.yAxis), 'b');
plot(handles.yAxis, min((((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - 1) / handles.frameRate) + rightTimeSegment), ((handles.frames - 1) / handles.frameRate)) * ones(1, 2), ylim(handles.yAxis), 'b');
plot(handles.zAxis, max((((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1) / handles.frameRate) - leftTimeSegment), 0) * ones(1, 2), ylim(handles.zAxis), 'b');
plot(handles.zAxis, min((((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - 1) / handles.frameRate) + rightTimeSegment), ((handles.frames - 1) / handles.frameRate)) * ones(1, 2), ylim(handles.zAxis), 'b');
plot(handles.eAxis, max((((handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) - 1) / handles.frameRate) - leftTimeSegment), 0) * ones(1, 2), ylim(handles.eAxis), 'b');
plot(handles.eAxis, min((((handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) - 1) / handles.frameRate) + rightTimeSegment), ((handles.frames - 1) / handles.frameRate)) * ones(1, 2), ylim(handles.eAxis), 'b');

function [dataSegment, constraints, error] = approximate(dataSegment, method, order)
% dataSegment   array of data with gap
% method        string of interpolation method
% order         order of polynomial approximation

% Compute approximation or interpolation for given data segment
dataIndex = ~isnan(dataSegment);
datax = find(~isnan(dataSegment));
datay = dataSegment(~isnan(dataSegment));
datady = zeros(2, 1);
datady(1) = (-datay(3) + 4 * datay(2) - 3 * datay(1)) / 2;
datady(2) = (3 * datay(end) - 4 * datay(end - 1) + datay(end - 2)) / 2;
dataX = (1:length(dataSegment))';
if strcmp(method, 'constrained fit')
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
    options = optimset('LargeScale', 'off', 'Display', 'off');
    p = lsqlin(C, d, [], [], A, b, [], [], [], options);
    dataSegment = polyval(p, dataX);
elseif strcmp(method, 'unconstrained fit')
    order = min(order, (length(datax) - 1));
    [p, S, m] = polyfit(datax, datay, order);
    dataSegment = polyval(p, dataX, S, m);
else
    dataSegment = interp1(datax, datay, dataX, method);
end

% Compute error and check constraints
error = sum((datay - dataSegment(dataIndex)).^2) / length(dataSegment);
constraints = zeros(2, 1);
if (abs(dataSegment(1) - datay(1)) > 0.1)
    constraints(1) = 1;
end
if (abs((((-dataSegment(3) + 4 * dataSegment(2) - 3 * dataSegment(1)) / 2) - datady(1)) / datady(1)) > 0.5)
    constraints(1) = 1;
end
if (abs(dataSegment(end) - datay(end)) > 0.1)
    constraints(2) = 1;
end
if (abs((((3 * dataSegment(end) - 4 * dataSegment(end - 1) + dataSegment(end - 2)) / 2) - datady(end)) / datady(end)) > 0.5)
    constraints(2) = 1;
end

function dataFileEdit_Callback(hObject, eventdata, handles)
% hObject    handle to dataFileEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function dataFileEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dataFileEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in browseButton.
function browseButton_Callback(hObject, eventdata, handles)
% hObject    handle to browseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[handles.dataFile, handles.dataPath, ~] = uigetfile('*.mat', 'Select the MAT data file', get(handles.dataFileEdit, 'String'));
if ~isnumeric(handles.dataFile) && (exist([handles.dataPath, handles.dataFile], 'file') == 2);
    % Set data file string
    set(handles.dataFileEdit, 'String', [handles.dataPath, handles.dataFile]);
    
    % Run data check
    handles = checkData(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes on button press in loadButton.
function loadButton_Callback(hObject, eventdata, handles)
% hObject    handle to loadButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if exist(get(handles.dataFileEdit, 'String'), 'file') == 2
    % Run data check
    handles = checkData(handles);
    
    % Update handles structure
    guidata(hObject, handles);
else
    msgbox('Data path is not valid!', 'ERROR', 'error');
end

function variableEdit_Callback(hObject, eventdata, handles)
% hObject    handle to variableEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function variableEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to variableEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when selected cell(s) is changed in gapsTable.
function gapsTable_CellSelectionCallback(hObject, eventdata, handles)
% hObject    handle to gapsTable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) currently selecteds
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(eventdata.Indices)
    if handles.gaps(eventdata.Indices(2)).number > 0    
        % Save selected trajectory index and reset gap index
        %set(handles.gapsTable, 'Enable', 'off');
        handles.trajectoryIndex = eventdata.Indices(2);
        handles.gapIndex = 1;

        % Plot gap
        handles = plotGap(handles);

        % Update handles structure
        guidata(hObject, handles);
    end
end

function frameEdit_Callback(hObject, eventdata, handles)
% hObject    handle to frameEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Ensure a numeric value
if isnan(str2double(get(handles.frameEdit, 'String'))) || (str2double(get(handles.frameEdit, 'String')) <= 0)
    set(handles.frameEdit, 'String', '1');
end

if ~isnan(handles.trajectoryIndex)
    % Update plot
    handles = plotGap(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function frameEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to frameEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function leftSegmentEdit_Callback(hObject, eventdata, handles)
% hObject    handle to leftSegmentEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Ensure a numeric value
if isnan(str2double(get(handles.leftSegmentEdit, 'String'))) || (str2double(get(handles.leftSegmentEdit, 'String')) <= 4)
    set(handles.leftSegmentEdit, 'String', '40');
end

% Round numeric value
set(handles.leftSegmentEdit, 'String', num2str(round(str2double(get(handles.leftSegmentEdit, 'String')))));

if ~isnan(handles.trajectoryIndex)
    % Update plot
    handles = plotGap(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function leftSegmentEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftSegmentEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function rightSegmentEdit_Callback(hObject, eventdata, handles)
% hObject    handle to rightSegmentEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Ensure a numeric value
if isnan(str2double(get(handles.rightSegmentEdit, 'String'))) || (str2double(get(handles.rightSegmentEdit, 'String')) <= 4)
    set(handles.rightSegmentEdit, 'String', '40');
end

% Round numeric value
set(handles.rightSegmentEdit, 'String', num2str(round(str2double(get(handles.rightSegmentEdit, 'String')))));

if ~isnan(handles.trajectoryIndex)
    % Update plot
    handles = plotGap(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function rightSegmentEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rightSegmentEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in methodMenu.
function methodMenu_Callback(hObject, eventdata, handles)
% hObject    handle to methodMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (get(handles.methodMenu, 'Value') == 1) || (get(handles.methodMenu, 'Value') == 2)
    set(handles.orderEdit, 'Enable', 'on');
else
    set(handles.orderEdit, 'Enable', 'off');
end

if ~isnan(handles.trajectoryIndex)
    % Update plot
    handles = plotGap(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function methodMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to methodMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function orderEdit_Callback(hObject, eventdata, handles)
% hObject    handle to orderEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Ensure a numeric value
if isnan(str2double(get(handles.orderEdit, 'String'))) || (str2double(get(handles.orderEdit, 'String')) < 4)
    set(handles.orderEdit, 'String', '4');
end

% Round numeric value
set(handles.orderEdit, 'String', num2str(round(str2double(get(handles.orderEdit, 'String')))));

if ~isnan(handles.trajectoryIndex)
    % Update plot
    handles = plotGap(handles);
    
    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function orderEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to orderEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in framePlusButton.
function framePlusButton_Callback(hObject, eventdata, handles)
% hObject    handle to framePlusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase time frame
set(handles.frameEdit, 'String', num2str(str2double(get(handles.frameEdit, 'String')) + 0.1));

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in frameMinusButton.
function frameMinusButton_Callback(hObject, eventdata, handles)
% hObject    handle to frameMinusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease time frame
if (str2double(get(handles.frameEdit, 'String')) > 0.1)
    set(handles.frameEdit, 'String', num2str(str2double(get(handles.frameEdit, 'String')) - 0.1));
end

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in rightSegmentPlusButton.
function rightSegmentPlusButton_Callback(hObject, eventdata, handles)
% hObject    handle to rightSegmentPlusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase right data segment
set(handles.rightSegmentEdit, 'String', num2str(str2double(get(handles.rightSegmentEdit, 'String')) + 5));

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in rightSegmentMinusButton.
function rightSegmentMinusButton_Callback(hObject, eventdata, handles)
% hObject    handle to rightSegmentMinusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease right data segment
if (str2double(get(handles.rightSegmentEdit, 'String')) > 5)
    set(handles.rightSegmentEdit, 'String', num2str(str2double(get(handles.rightSegmentEdit, 'String')) - 5));
end

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in leftSegmentPlusButton.
function leftSegmentPlusButton_Callback(hObject, eventdata, handles)
% hObject    handle to leftSegmentPlusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase left data segment
set(handles.leftSegmentEdit, 'String', num2str(str2double(get(handles.leftSegmentEdit, 'String')) + 5));

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in leftSegmentMinusButton.
function leftSegmentMinusButton_Callback(hObject, eventdata, handles)
% hObject    handle to leftSegmentMinusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease left data segment
if (str2double(get(handles.leftSegmentEdit, 'String')) > 5)
    set(handles.leftSegmentEdit, 'String', num2str(str2double(get(handles.leftSegmentEdit, 'String')) - 5));
end

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in orderPlusButton.
function orderPlusButton_Callback(hObject, eventdata, handles)
% hObject    handle to orderPlusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Increase order
set(handles.orderEdit, 'String', num2str(str2double(get(handles.orderEdit, 'String')) + 1));

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in orderMinusButton.
function orderMinusButton_Callback(hObject, eventdata, handles)
% hObject    handle to orderMinusButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Decrease order
if (str2double(get(handles.orderEdit, 'String')) > 1)
    set(handles.orderEdit, 'String', num2str(str2double(get(handles.orderEdit, 'String')) - 1));
end

% Update plot
if ~isnan(handles.trajectoryIndex)
    handles = plotGap(handles);
end

% Update handles structure
guidata(hObject, handles);

% --- Executes on button press in acceptButton.
function acceptButton_Callback(hObject, eventdata, handles)
% hObject    handle to acceptButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isnan(handles.trajectoryIndex)
    % Store appriximation in data structure
    startIndex = handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex);
    endIndex = handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex);
    handles.data(handles.trajectoryIndex, 1, startIndex:endIndex) = handles.xApproximation(startIndex:endIndex);
    handles.data(handles.trajectoryIndex, 2, startIndex:endIndex) = handles.yApproximation(startIndex:endIndex);
    handles.data(handles.trajectoryIndex, 3, startIndex:endIndex) = handles.zApproximation(startIndex:endIndex);
    %handles.data(handles.trajectoryIndex, 4, startIndex:endIndex) = handles.eApproximation(startIndex:endIndex);

    % Update gap data structure and table
    handles.gaps(handles.trajectoryIndex).startIndex(handles.gapIndex) = [];
    handles.gaps(handles.trajectoryIndex).endIndex(handles.gapIndex) = [];
    handles.gaps(handles.trajectoryIndex).number = handles.gaps(handles.trajectoryIndex).number - 1;
    if handles.gaps(handles.trajectoryIndex).number > 0
        handles.gaps(handles.trajectoryIndex).formatString = ['<html><span style="color:#AA0000; font-weight:bold;">', num2str(handles.gaps(handles.trajectoryIndex).number), '</span></html>'];
    else
        handles.gaps(handles.trajectoryIndex).formatString = ['<html><span style="color:#00AA00;">', num2str(handles.gaps(handles.trajectoryIndex).number), '</span></html>'];
    end
    set(handles.gapsTable, 'Data', {handles.gaps(:).formatString});

    % Plot next gap or clear plot
    if handles.gaps(handles.trajectoryIndex).number > 0
        handles = plotGap(handles);
    else
        % Clear GUI
        set(handles.infoLabel, 'String', 'No data');

        % Clear plot
        cla(handles.xAxis);
        cla(handles.yAxis);
        cla(handles.zAxis);
        cla(handles.eAxis);
        
        % Clear trajectory index
        handles.trajectoryIndex = nan;
    end
    
    % Update handles structure
    guidata(hObject, handles);
end



% --- Executes on button press in skipForthButton.
function skipForthButton_Callback(hObject, eventdata, handles)
% hObject    handle to skipForthButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isnan(handles.trajectoryIndex)
    % Increment gap index
    handles.gapIndex = min((handles.gapIndex + 1), handles.gaps(handles.trajectoryIndex).number);

    % Plot gap
    handles = plotGap(handles);

    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes on button press in skipBackButton.
function skipBackButton_Callback(hObject, eventdata, handles)
% hObject    handle to skipBackButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isnan(handles.trajectoryIndex)
    % Decrement gap index
    handles.gapIndex = max((handles.gapIndex - 1), 1);

    % Plot gap
    handles = plotGap(handles);

    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes on button press in saveButton.
function saveButton_Callback(hObject, eventdata, handles)
% hObject    handle to saveButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.dataLoaded
    dataFile = get(handles.dataFileEdit, 'String');
    if exist(dataFile, 'file') == 2
        % Save modified data to new file and adjust reference frame
        % according to [Wu2002], [Wu2005] (x -> z, y -> x, z -> y)
        [path, file, extention] = fileparts(dataFile);
        file = regexp(file, ' ', 'split');
        file = file(2:end);
        file(2, :) = {' '};
        file = [file{:}];
        file = file(1:(end - 1));
        saveFile = [path, filesep, '..', filesep, file, extention];
        motion.frameRate = handles.frameRate;
        motion.frames = handles.frames;
        motion.markerLabels = handles.labels;
        motion.markerX = squeeze(handles.data(:, 2, :));
        motion.markerY = squeeze(handles.data(:, 3, :));
        motion.markerZ = squeeze(handles.data(:, 1, :));
        motion.markerE = squeeze(handles.data(:, 4, :));
        if exist(saveFile, 'file') == 2
            save([path, filesep, '..', filesep, file, extention], 'motion', '-append');
        else
            save([path, filesep, '..', filesep, file, extention], 'motion');
        end
        msgbox(['Data was saved to "', file, extention, '" in parent directory.'], 'INFO');
    else
        msgbox('Data path is not valid!', 'ERROR', 'error');
    end
end

% --- Executes on button press in abortButton.
function abortButton_Callback(hObject, eventdata, handles)
% hObject    handle to abortButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isnan(handles.trajectoryIndex)
    % Clear table
    set(handles.gapsTable, 'ColumnName', []);
    set(handles.gapsTable, 'Data', []);
    set(handles.gapsTable, 'Enable', 'off');

    % Clear GUI
    set(handles.infoLabel, 'String', 'No data');

    % Clear plot
    cla(handles.xAxis);
    cla(handles.yAxis);
    cla(handles.zAxis);
    cla(handles.eAxis);
    
    % Clear trajectory index
    handles.trajectoryIndex = nan;
    
    % Clear data loaded flag
    handles.dataLoaded = 0;

    % Update handles structure
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function errorEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to errorEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in leftConstraintsRadio.
function leftConstraintsRadio_Callback(hObject, eventdata, handles)
% hObject    handle to leftConstraintsRadio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function leftConstraintsEdit_Callback(hObject, eventdata, handles)
% hObject    handle to leftConstraintsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function leftConstraintsEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to leftConstraintsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function rightConstraintsEdit_Callback(hObject, eventdata, handles)
% hObject    handle to rightConstraintsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function rightConstraintsEdit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rightConstraintsEdit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in mirrorCheckbox.
function mirrorCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to mirrorCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Plot gap
handles = plotGap(handles);

% Update handles structure
guidata(hObject, handles);
