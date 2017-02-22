function varargout = simple_gui(varargin)
% SIMPLE_GUI MATLAB code for simple_gui.fig      
% Last Modified by GUIDE v2.5 22-Feb-2017 09:26:28

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simple_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @simple_gui_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before simple_gui is made visible.
function simple_gui_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simple_gui (see VARARGIN)

% Choose default command line output for simple_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
            
% UIWAIT makes simple_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = simple_gui_OutputFcn(hObject, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in stop.
function stop_Callback(hObject, ~, handles)
% hObject    handle to stop (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcbf)
simple_gui

% --- Executes on button press in start.
function start_Callback(hObject, ~, handles)
% hObject    handle to start (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
sx=str2double(get(handles.start_x,'String'));
sy=str2double(get(handles.start_y,'String'));
gx=str2double(get(handles.goal_x,'String'));
gy=str2double(get(handles.goal_y,'String'));
S=[sx;sy];
G=[gx;gy];

% Mark start and goal positions
scatter(S(1),S(2),'g','filled');
scatter(G(1),G(2),'r','filled');

plan=get(handles.planner,'Value');

if isequal(plan,1)
    T=RRT(S,G,40,40);
    Obstacles=drawObstacles;
    makeTree(Obstacles,T);
end
if isequal(plan,2)
    T1=RRT(S,G,40,40);
    T2=RRT(G,S,40,40);
    Obstacles=drawObstacles;
    makeTree(Obstacles,T1,T2);
end

% --- Executes during object creation, after setting all properties.
function workspace_axes_CreateFcn(hObject, ~, handles)
% hObject    handle to workspace_axes (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

%draw workspace_axes with obstacles
workspace(1000);
o=drawObstacles;

function start_x_Callback(hObject, ~, handles)
% hObject    handle to start_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s_x=str2double(get(hObject, 'string'));
if(isempty(s_x))
    set(hObject,'String','0');
end
%setappdata(0,'s_x',s_x);

% --- Executes during object creation, after setting all properties.
function start_x_CreateFcn(hObject, ~, handles)
% hObject    handle to start_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function start_y_Callback(hObject, ~, handles)
% hObject    handle to start_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s_y=str2double(get(hObject, 'string'));
if(isempty(s_y))
    set(hObject,'String','0');
end
%setappdata(0,'s_y',s_y);

% --- Executes during object creation, after setting all properties.
function start_y_CreateFcn(hObject, ~, handles)
% hObject    handle to start_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function goal_x_Callback(hObject, ~, handles)
% hObject    handle to goal_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

g_x=str2double(get(hObject, 'string'));
if(isempty(g_x))
    set(hObject,'String','0');
end

% --- Executes during object creation, after setting all properties.
function goal_x_CreateFcn(hObject, ~, handles)
% hObject    handle to goal_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function goal_y_Callback(hObject, ~, handles)
% hObject    handle to goal_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

g_y=str2double(get(hObject, 'string'));
if(isempty(g_y))
    set(hObject,'String','0');
end

% --- Executes during object creation, after setting all properties.
function goal_y_CreateFcn(hObject, ~, handles)
% hObject    handle to goal_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pointer_start.
function pointer_start_Callback(hObject, eventdata, handles)
% hObject    handle to pointer_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
set(handles.start_x,'String',num2str(x));
set(handles.start_y,'String',num2str(y));

% --- Executes on button press in pointer_goal.
function pointer_goal_Callback(hObject, eventdata, handles)
% hObject    handle to pointer_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[x,y]=ginput(1);
set(handles.goal_x,'String',num2str(x));
set(handles.goal_y,'String',num2str(y));

% --- Executes on selection change in planner.
function planner_Callback(hObject, eventdata, handles)
% hObject    handle to planner (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns planner contents as cell array
%        contents{get(hObject,'Value')} returns selected item from planner
plan=get(hObject,'Value');

% --- Executes during object creation, after setting all properties.
function planner_CreateFcn(hObject, eventdata, handles)
% hObject    handle to planner (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
