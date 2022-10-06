function varargout = AdvTeach(varargin)
% ADVTEACH MATLAB code for AdvTeach.fig
%      ADVTEACH, by itself, creates a new ADVTEACH or raises the existing
%      singleton*.
%
%      H = ADVTEACH returns the handle to a new ADVTEACH or the handle to
%      the existing singleton*.
%
%      ADVTEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ADVTEACH.M with the given input arguments.
%
%      ADVTEACH('Property','Value',...) creates a new ADVTEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AdvTeach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AdvTeach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AdvTeach

% Last Modified by GUIDE v2.5 06-Oct-2022 20:46:50

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

axes(handles.axes1);
L(1) = Link([0 0.34 0 -pi/2]); % 0.33997
L(2) = Link([0 0 0 pi/2]);
L(3) = Link([0 0.4 0 pi/2]);
L(4) = Link([0 0 0 -pi/2]);
L(5) = Link([0 0.4 0 -pi/2]); % 0.39998
L(6) = Link([0 0 0 pi/2]);
L(7) = Link([0 0.126 0 0]);

% Incorporate joint limits
L(1).qlim = [-170 170]*pi/180;
L(2).qlim = [-120 120]*pi/180;
L(3).qlim = [-170 170]*pi/180;
L(4).qlim = [-120 120]*pi/180;
L(5).qlim = [-170 170]*pi/180;
L(6).qlim = [-120 120]*pi/180;
L(7).qlim = [-175 175]*pi/180;
model = SerialLink(L,'name','iiwa');

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri');
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


% --- Executes on button press in plusY.
function plusY_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in minusX.
function minusX_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);



% --- Executes on button press in minusY.
function minusY_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in plusX.
function plusX_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in minusZ.
function minusZ_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on button press in plusZ.
function plusZ_Callback(hObject, eventdata, handles)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.1;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


% --- Executes on slider movement.
function link1Slider_Callback(hObject, eventdata, handles)
% hObject    handle to link1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(hObject,'Min',-170)
set(hObject,'Max',170)
val = get(hObject, 'Value');
link1 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([link1,currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link1Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link1Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link2Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-120)
set(hObject,'Max',120)
val = get(hObject, 'Value');
link2 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),link2,currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link2Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link2Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link3Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-170)
set(hObject,'Max',170)
val = get(hObject, 'Value');
link3 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),link3,currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link3Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link3Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link4Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-120)
set(hObject,'Max',120)
val = get(hObject, 'Value');
link4 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),link4,currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link4Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link4Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link5Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-170)
set(hObject,'Max',170)
val = get(hObject, 'Value');
link5 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),link5,currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link5Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link5Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link6Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-120)
set(hObject,'Max',120)
val = get(hObject, 'Value');
link6 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),link6,currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function link6Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link6Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function link7Slider_Callback(hObject, eventdata, handles)
set(hObject,'Min',-175)
set(hObject,'Max',175)
val = get(hObject, 'Value');
link7 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,6),link7]);


% --- Executes during object creation, after setting all properties.
function link7Slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to link7Slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
val = str2double(get(hObject,'String'));
link1 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([link1,currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);



% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link2 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),link2,currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link3 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,3),link3,currentQ(1,4),currentQ(1,5),currentQ(1,6),currentQ(1,7)]);

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link4 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),link4,currentQ(1,5),currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link5 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),link5,currentQ(1,6),currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit7_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link6 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),link6,currentQ(1,7)]);


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit8_Callback(hObject, eventdata, handles)
val = str2double(get(hObject,'String'));
link7 = deg2rad(val);
currentQ = handles.model.getpos();
handles.model.animate([currentQ(1,1),currentQ(1,2),currentQ(1,3),currentQ(1,4),currentQ(1,5),currentQ(1,7),link7]);


% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
