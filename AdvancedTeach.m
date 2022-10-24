function varargout = AdvancedTeach(varargin)
% ADVANCEDTEACH MATLAB code for AdvancedTeach.fig
%      ADVANCEDTEACH, by itself, creates a new ADVANCEDTEACH or raises the existing
%      singleton*.
%
%      H = ADVANCEDTEACH returns the handle to a new ADVANCEDTEACH or the handle to
%      the existing singleton*.
%
%      ADVANCEDTEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ADVANCEDTEACH.M with the given input arguments.
%
%      ADVANCEDTEACH('Property','Value',...) creates a new ADVANCEDTEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AdvancedTeach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AdvancedTeach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AdvancedTeach

% Last Modified by GUIDE v2.5 24-Oct-2022 11:08:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @AdvTeach_OpeningFcn, ...
    'gui_OutputFcn',  @AdvTeach_OutputFcn, ...
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

% --- Executes just before AdvTeach is made visible.
function AdvTeach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AdvTeach (see VARARGIN)

% Choose default command line output for AdvTeach
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using AdvTeach.
if strcmp(get(hObject,'Visible'),'off')
    plot(0);
end

L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
L7 = Link('d',0,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

model = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3');
for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['ur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end
% Display robot
workspace = [-2 2 -2 2 -0.3 2];
model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
model.delay = 0;
% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
data = guidata(hObject);
data.model = model;
guidata(hObject,data);

% UIWAIT makes AdvTeach wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AdvTeach_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
    ['Close ' get(handles.figure1,'Name') '...'],...
    'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});





% --- Executes on button press in minusX.
function minusX_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in plusX.
function plusX_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in minusY.
function minusY_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in plusY.
function plusY_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in plusZ.
function plusZ_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in minusZ.
function minusZ_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in Simulation.
function Simulation_Callback(hObject, eventdata, handles)
set(handles.eStop, 'userdata', 1);
set(handles.Resume,'userdata',1)
x = 1

while x < 10
    drawnow;
    
    Stop = get(handles.eStop,'userdata');
    Resume = get(handles.Resume,'userdata')
    
    if Stop == 0
        disp('break');
    end;
    
        % demo simulation
        q = handles.model.getpos;
        tr1 = transl([ -1,0.1,0.1]);
        tr2 = transl([ -0.1,-0.1,-0.1])
        q1 = handles.model.ikcon(tr1)
        q2 = handles.model.ikcon(tr2)
        steps = 300
        qMatrix = jtraj(q,q1,steps);
        for i = 1:steps;
            handles.model.animate(qMatrix(i,:))
            drawnow();
        end
        qMatrix = jtraj(q1,q2,steps)
        for i = 1:steps;
            handles.model.animate(qMatrix(i,:))
            drawnow();
        end
        
        
        x = x+1
end
   


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over Simulation.
% function Simulation_ButtonDownFcn(hObject, eventdata, handles)
% % hObject    handle to Simulation (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% q = handles.model.getpos
% tr = handles.model.fkine(q);
% handles.model.animate(tr)
% drawnow();


% --- Executes on button press in eStop.
function eStop_Callback(hObject, eventdata, handles)
if get(handles.eStop,'userdata')==1

% msgfig1 = msgbox('Operation has been halted!','Success','modal');
% uiwait(msgfig);
uiwait();
set(handles.eStop,'userdata',0);
% if get(handles.Resume,'userdata')==1
%     uiresume()
 else
 set(handles.eStop,'userdata',1);
end
    


% --- Executes on button press in Resume.
function Resume_Callback(hObject, eventdata, handles)
msgfig = msgbox('Click OK if you wish to Resume!','Success','modal');

if get(handles.eStop,'userdata')==1;
uiwait(msgfig)
uiresume()
set(handles.eStop,'userdata',0);
else
set(handles.eStop,'userdata',1);
end

