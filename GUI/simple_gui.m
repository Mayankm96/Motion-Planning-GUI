function varargout = simple_gui(varargin)
% SIMPLE_GUI MATLAB code for simple_gui.fig      
% Last Modified by GUIDE v2.5 05-Mar-2017 14:57:38
addpath('Map');
addpath('RRTree');
addpath('PRM');

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
function varargout = simple_gui_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in stop.
function stop_Callback(~, ~, ~)
% hObject    handle to stop (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(0,'showhiddenhandles','on');    % Make the GUI figure handle visible
h=findobj(gcf,'type','axes');       % Find the axes object in the GUI
f=figure;                           % Open a new figure with handle f
s=copyobj(h,f);                    % Copy axes object h into figure f
axis equal;

% --- Executes on button press in start.
function start_Callback(~, ~, handles)
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
pause(0.005);
plan=get(handles.planner,'Value');
Obstacles=getappdata(handles.make_map,'Obstacles');
flag=get(handles.show_path,'Value');

switch plan
    case 1
    T=RRT(S,G,20,Obstacles);
    T.makeTree('b',flag);

    case 2
    T1=RRT(S,G,20,Obstacles);
    T2=RRT(G,S,20,Obstacles);
    T1.makeTree('g',flag,T2,'r');
    
    case 3
    M=PRM(S,G,20,Obstacles);
    M.makePRM('m',flag);
end

% --- Executes during object creation, after setting all properties.
function workspace_axes_CreateFcn(~, ~, ~)
% hObject    handle to workspace_axes (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

%draw workspace_axes with obstacles
workspace(1000);

function start_x_Callback(hObject, ~, ~)
% hObject    handle to start_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s_x=str2double(get(hObject, 'string'));
if(isempty(s_x))
    set(hObject,'String','50');
end

% --- Executes during object creation, after setting all properties.
function start_x_CreateFcn(hObject, ~, ~)
% hObject    handle to start_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function start_y_Callback(hObject, ~, ~)
% hObject    handle to start_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

s_y=str2double(get(hObject, 'string'));
if(isempty(s_y))
    set(hObject,'String','50');
end

% --- Executes during object creation, after setting all properties.
function start_y_CreateFcn(hObject, ~, ~)
% hObject    handle to start_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function goal_x_Callback(hObject, ~, ~)
% hObject    handle to goal_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

g_x=str2double(get(hObject, 'string'));
if(isempty(g_x))
    set(hObject,'String','950');
end

% --- Executes during object creation, after setting all properties.
function goal_x_CreateFcn(hObject, ~, ~)
% hObject    handle to goal_x (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function goal_y_Callback(hObject, ~, ~)
% hObject    handle to goal_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

g_y=str2double(get(hObject, 'string'));
if(isempty(g_y))
    set(hObject,'String','950');
end

% --- Executes during object creation, after setting all properties.
function goal_y_CreateFcn(hObject, ~, ~)
% hObject    handle to goal_y (see GCBO)
% ~  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pointer_start.
function pointer_start_Callback(~, ~, handles)
% hObject    handle to pointer_start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% delete the previously marked start point (if any)
if isappdata(handles.pointer_start,'fig')
    n=getappdata(handles.pointer_start,'fig');
    delete(n);
end

[n,x,y]=ginputp('og',1);
set(handles.start_x,'String',num2str(x));
set(handles.start_y,'String',num2str(y));
setappdata(handles.pointer_start,'fig',n);

% --- Executes on button press in pointer_goal.
function pointer_goal_Callback(~, ~, handles)
% hObject    handle to pointer_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% delete the previously marked goal point (if any)
if isappdata(handles.pointer_goal,'fig')
    n=getappdata(handles.pointer_goal,'fig');
    delete(n);
end

[n,x,y]=ginputp('or',1);
set(handles.goal_x,'String',num2str(x));
set(handles.goal_y,'String',num2str(y));
setappdata(handles.pointer_goal,'fig',n);

% --- Executes on selection change in planner.
function planner_Callback(hObject, ~, ~)
% hObject    handle to planner (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns planner contents as cell array
%        contents{get(hObject,'Value')} returns selected item from planner
plan=get(hObject,'Value');

% --- Executes during object creation, after setting all properties.
function planner_CreateFcn(hObject, ~, ~)
% hObject    handle to planner (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in mapper.
function mapper_Callback(hObject, ~, ~)
% hObject    handle to mapper (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mapper contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mapper
map=get(hObject,'Value');


% --- Executes during object creation, after setting all properties.
function mapper_CreateFcn(hObject, ~, ~)
% hObject    handle to mapper (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in make_map.
function make_map_Callback(~, ~, handles)
% hObject    handle to make_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
map=get(handles.mapper,'Value');
set(handles.addob,'Visible','off');

cla reset
workspace(1000);
switch map
    case 1
        Obstacles=[];
    case 2
        Obstacles=drawObstacles(2);
    case 3
        Obstacles=drawObstacles(3);
    case 4
        set(handles.addob,'Visible','on');
        setappdata(handles.addob,'Value',0);
        set(handles.addob,'String','O');

        hold on;
        i=1;
        while 1==1
            [~,X,Y] = ginputp('.k');
            poly=fill(X,Y,'k');
            Obstacles{i}=struct('Vertices',poly.Vertices,'FaceColor',poly.FaceColor);
            i=i+1;
            if getappdata(handles.addob,'Value')
                break;
            end
        end
        %Obstacles=Obstacles(1,1:(length(Obstacles)-2));
        set(handles.addob,'String','X');
end
setappdata(handles.make_map,'Obstacles',Obstacles);


% --- Executes on button press in addob.
function addob_Callback(~, ~, handles)
% hObject    handle to addob (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(handles.addob,'Value',1);


% --- Executes on button press in show_path.
function show_path_Callback(hObject, ~, ~)
% hObject    handle to show_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
path=get(hObject,'Value');
