function varargout = ImoovCalibration(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ImoovCalibration_OpeningFcn, ...
                   'gui_OutputFcn',  @ImoovCalibration_OutputFcn, ...
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

function ImoovCalibration_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
guidata(hObject, handles);
global state muestras
state = false;
muestras = 2000;


% --- Outputs from this function are returned to the command line.
function varargout = ImoovCalibration_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

% --- Executes on button press in MagData.
function MagData_Callback(hObject, eventdata, handles)
global mx my mz s muestras numSensor
grid(handles.axes1,'on');
xlabel(handles.axes1, 'Magnetometro X');
ylabel(handles.axes1, 'Magnetometro Y');
zlabel(handles.axes1, 'Magnetometro Z');
title(handles.axes1,'Magnetometer');
disp('inicio');
for i=2:muestras
    if(numSensor==1)
        fwrite(s,'c','char');
    elseif(numSensor==2)
        fwrite(s,'f','char');   
    elseif(numSensor==3)
        fwrite(s,'i','char');
    elseif(numSensor==4)
        fwrite(s,'l','char');
    end
    
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    mx(i)=a(1);
    my(i)=a(2);
    mz(i)=a(3);
    
        plot3(handles.axes1,mx(i),my(i),mz(i),'*');
        hold on;
        axis equal;
        drawnow
end
disp('listo');
hold(handles.axes1,'off');


% --- Executes on button press in CalMag.
function CalMag_Callback(hObject, eventdata, handles)
global mx my mz muestras 
%Descartamos las primeras medidas erroneas
a=10;

%Definimos los vectores de medición
x=mx(a:muestras);
y=my(a:muestras);
z=mz(a:muestras);
% x2=x.^2;
% y2=y.^2;
% z2=z.^2;
% xy=x.*y;
% xz=x.*z;
% yz=y.*z;

x2=x.^2;
y2=-y.^2;
z2=-z.^2;
save variablesMag x y z


% D=[x2' y2' z2' 2*xy' 2*xz' 2*yz' 2*x' 2*y' 2*z' ];
% v=inv(D'*D)*D'*ones(size(x,2),1);
% Vghi=v(7:9);
% A4=[v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -1];
% A3=[v(1) v(4) v(5); v(4) v(2) v(6); v(5) v(6) v(3)];
% 
% Offsets= -inv(A3)*Vghi;
% 
% T=[1 0 0 0; 0 1 0 0; 0 0 1 0; Offsets(1) Offsets(2) Offsets(3) 1];
% B4=T*A4*T';
% B3=B4(1:3,1:3)/-B4(4,4);
% [rotM ev]=eig(B3);
% gain=sqrt(1./diag(ev));
% 
% %% Señales Calibradas
% XC=x-Offsets(1);
% YC=y-Offsets(2);
% ZC=z-Offsets(3);
% 
% XYZC=[XC' YC' ZC']*rotM;
% XC=XYZC(:,1)/gain(1);
% YC=XYZC(:,2)/gain(2);
% ZC=XYZC(:,3)/gain(3);

%% Calibracion basica
%Ecuacion W=H*X ==>> X=([Ht*H]^-1)*Ht*W
%W de tamaño Nx1  con N=numero de muestra
%H de tamaño N*6 
%X de tamaño 6*1 Representa variables que contienen informacion sobre el
%escalamiento y offsets de la medición
W=x2';
H=[x' y' z' y2' z2' ones(size(x'))];
X=inv((H')*H)*H'*W;

% Recuperamos la informacion de los offset y escalas en cada eje

Xoffset=X(1)/2;
Yoffset=X(2)/(2*X(4));
Zoffset=X(3)/(2*X(5));
A=X(6)+Xoffset^2+X(4)*Yoffset^2+X(5)*Zoffset^2;
B=A/X(4);
C=A/X(5);

xx=x-Xoffset;
yy=y-Yoffset;
zz=z-Zoffset;
%Ahora nuestra ecuacion que represeta la esfera formada por las medidas de
%los sensores es: xx/A + yy/B + zz/C = 1

XScale=A^(1/2);
YScale=B^(1/2);
ZScale=C^(1/2);

% Y por ultimo nuestras medidas son:
xxx=xx/XScale;
yyy=yy/YScale;
zzz=zz/ZScale;

disp(Xoffset);
disp(Yoffset);
disp(Zoffset);
disp(XScale);
disp(YScale);
disp(ZScale);

%% Plot
grid(handles.axes1,'on');
%plot3(handles.axes1,XC,YC,ZC,'b');
plot3(handles.axes1,xxx,yyy,zzz,'b');
axis equal;
xlabel(handles.axes1, 'Magnetometro X');
ylabel(handles.axes1, 'Magnetometro Y');
zlabel(handles.axes1, 'Magnetometro Z');
hold(handles.axes1,'off');
title(handles.axes1,'Magnetometer');

% eax=strcat('mx = ',num2str(rotM(1,1)),'*MagX',' + ',num2str(rotM(2,1)),'*MagY',' + ',num2str(rotM(3,1)),'*MagZ',' / ',num2str(gain(1)) );
% eay=strcat('my = ',num2str(rotM(1,2)),'*MagX',' + ',num2str(rotM(2,2)),'*MagY',' + ',num2str(rotM(3,2)),'*MagZ',' / ',num2str(gain(2)) );
% eaz=strcat('mz = ',num2str(rotM(1,3)),'*MagX',' + ',num2str(rotM(2,3)),'*MagY',' + ',num2str(rotM(3,3)),'*MagZ',' / ',num2str(gain(3)) );
% 
% set(handles.omx,'String',Offsets(1)-1000);
% set(handles.omy,'String',Offsets(2)-1000);
% set(handles.omz,'String',Offsets(3)-1000);
% 
% set(handles.EcuacionMagX,'String',eax);
% set(handles.EcuacionMagY,'String',eay);
% set(handles.EcuacionMagZ,'String',eaz);
% 
% disp(eax);
% disp(eay);
% disp(eaz);
% 
% disp(rotM)

%% ----------------------------------ACELEROMETRO --------------------------------
%%--------------------------------------------------------------------------------

function AxPData_Callback(hObject, eventdata, handles)
global ax s muestras numSensor
muestras = 3000;
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(1,i)=a(1);
    ax(2,i)=a(2);
    ax(3,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(1,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(2,:),'g');
plot(handles.axes1,ax(3,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (X+) [m/s^2]');
hold(handles.axes1,'off');

% --- Executes on button press in AxNData.
function AxNData_Callback(hObject, eventdata, handles)
global ax s muestras numSensor
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(4,i)=a(1);
    ax(5,i)=a(2);
    ax(6,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(4,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(5,:),'g');
plot(handles.axes1,ax(6,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (X-) [m/s^2]');
hold(handles.axes1,'off');

% --- Executes on button press in AyPData.
function AyPData_Callback(hObject, eventdata, handles)
global ax s muestras numSensor
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(7,i)=a(1);
    ax(8,i)=a(2);
    ax(9,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(7,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(8,:),'g');
plot(handles.axes1,ax(9,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (Y+) [m/s^2]');
hold(handles.axes1,'off');


% --- Executes on button press in AyNData.
function AyNData_Callback(hObject, eventdata, handles)
global ax s muestras numSensor
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(10,i)=a(1);
    ax(11,i)=a(2);
    ax(12,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(10,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(11,:),'g');
plot(handles.axes1,ax(12,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (Y-) [m/s^2]');
hold(handles.axes1,'off');

% --- Executes on button press in AzPData.
function AzPData_Callback(hObject, eventdata, handles)
global ax s muestras numSensor
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(13,i)=a(1);
    ax(14,i)=a(2);
    ax(15,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(13,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(14,:),'g');
plot(handles.axes1,ax(15,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (Z-) [m/s^2]');
hold(handles.axes1,'off');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (Z-) [m/s^2]');
hold(handles.axes1,'off');


% --- Executes on button press in AzNData.
function AzNData_Callback(hObject, eventdata, handles)

global ax s muestras numSensor
for i=1:muestras
    if(numSensor==1)
        fwrite(s,'a','char');
    elseif(numSensor==2)
        fwrite(s,'d','char');   
    elseif(numSensor==3)
        fwrite(s,'g','char');
    elseif(numSensor==4)
        fwrite(s,'j','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    ax(16,i)=a(1);
    ax(17,i)=a(2);
    ax(18,i)=a(3);
end
grid(handles.axes1,'on');
plot(handles.axes1,ax(16,:),'r');
hold(handles.axes1,'on');
plot(handles.axes1,ax(17,:),'g');
plot(handles.axes1,ax(18,:),'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Acelerometro sin Calibrar (Z-) [m/s^2]');
hold(handles.axes1,'off');

% --- Executes on button press in CalAccel.
function CalAccel_Callback(hObject, eventdata, handles)

global ax muestras
for i=1:18
W(i)=median(ax(i,:));
end

% Wf=[W(1) W(2) W(3) 1;
%     W(4) W(5) W(6) 1;
%     W(7) W(8) W(9) 1
%     W(10) W(11) W(12) 1;
%     W(13) W(14) W(15) 1;
%     W(16) W(17) W(18) 1];

Y2=[1 0 0;-1 0 0;0 1 0;0 -1 0;0 0 1;0 0 -1];
for i=1:6
    for j=1:muestras
        for k=1:3
            Wf((i-1)*muestras+j,k)=ax((i-1)*3+k,j);
            Y((i-1)*muestras+j,k)=Y2(i,k);
        end
        Wf((i-1)*muestras+j,4)=1;
    end
end
%Y=Y*4096;
%Y=[0 0 1;0 0 -1;0 -1 0;0 1 0;1 0 0;-1 0 0]*4096;
X=inv(Wf'*Wf)*Wf'*Y;

eax=strcat('ax = ',num2str(X(1,1)),'*AccelX',' + ',num2str(X(2,1)),'*AccelY',' + ',num2str(X(3,1)),'*AccelZ',' + ',num2str(X(4,1)) );
eay=strcat('ay = ',num2str(X(1,2)),'*AccelX',' + ',num2str(X(2,2)),'*AccelY',' + ',num2str(X(3,2)),'*AccelZ',' + ',num2str(X(4,2)) );
eaz=strcat('az = ',num2str(X(1,3)),'*AccelX',' + ',num2str(X(2,3)),'*AccelY',' + ',num2str(X(3,3)),'*AccelZ',' + ',num2str(X(4,3)) );

set(handles.EcuacionAcelX,'String',eax);
set(handles.EcuacionAcelY,'String',eay);
set(handles.EcuacionAcelZ,'String',eaz);

disp(eax);
disp(eay);
disp(eaz);

a(1)=X(1,1)*W(1)+X(2,1)*W(2)+X(3,1)*W(3)+X(4,1);
a(2)=X(1,2)*W(1)+X(2,2)*W(2)+X(3,2)*W(3)+X(4,2);
a(3)=X(1,3)*W(1)+X(2,3)*W(2)+X(3,3)*W(3)+X(4,3);

a(4)=X(1,1)*W(4)+X(2,1)*W(5)+X(3,1)*W(6)+X(4,1);
a(5)=X(1,2)*W(4)+X(2,2)*W(5)+X(3,2)*W(6)+X(4,2);
a(6)=X(1,3)*W(4)+X(2,3)*W(5)+X(3,3)*W(6)+X(4,3);

a(7)=X(1,1)*W(7)+X(2,1)*W(8)+X(3,1)*W(9)+X(4,1);
a(8)=X(1,2)*W(7)+X(2,2)*W(8)+X(3,2)*W(9)+X(4,2);
a(9)=X(1,3)*W(7)+X(2,3)*W(8)+X(3,3)*W(9)+X(4,3);

a(10)=X(1,1)*W(10)+X(2,1)*W(11)+X(3,1)*W(12)+X(4,1);
a(11)=X(1,2)*W(10)+X(2,2)*W(11)+X(3,2)*W(12)+X(4,2);
a(12)=X(1,3)*W(10)+X(2,3)*W(11)+X(3,3)*W(12)+X(4,3);

a(13)=X(1,1)*W(13)+X(2,1)*W(14)+X(3,1)*W(15)+X(4,1);
a(14)=X(1,2)*W(13)+X(2,2)*W(14)+X(3,2)*W(15)+X(4,2);
a(15)=X(1,3)*W(13)+X(2,3)*W(14)+X(3,3)*W(15)+X(4,3);

a(16)=X(1,1)*W(16)+X(2,1)*W(17)+X(3,1)*W(18)+X(4,1);
a(17)=X(1,2)*W(16)+X(2,2)*W(17)+X(3,2)*W(18)+X(4,2);
a(18)=X(1,3)*W(16)+X(2,3)*W(17)+X(3,3)*W(18)+X(4,3);

disp(a);

%% ----------------------------------GIROSCOPIO --------------------------------
%%--------------------------------------------------------------------------------

function GyroData_Callback(hObject, eventdata, handles)
global gx gy gz s muestras numSensor

for i=1:muestras
    if(numSensor==1)
        fwrite(s,'b','char');
    elseif(numSensor==2)
        fwrite(s,'e','char');   
    elseif(numSensor==3)
        fwrite(s,'h','char');
    elseif(numSensor==4)
        fwrite(s,'k','char');
    end
    % leer del puerto serie
    a = fscanf(s,'%d,%d,%d')';
    gx(i)=a(1);
    gy(i)=a(2);
    gz(i)=a(3);
end

grid(handles.axes1,'on');
plot(handles.axes1,gx,'r');
hold(handles.axes1,'on');
plot(handles.axes1,gy,'g');
plot(handles.axes1,gz,'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Giroscopio sin Calibrar [rad/s]');
hold(handles.axes1,'off');


% --- Executes on button press in CalGyro.
function CalGyro_Callback(hObject, eventdata, handles)
% hObject    handle to CalGyro (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global gx gy gz muestras 
MeanGX=mean(gx);
MeanGY=mean(gy);
MeanGZ=mean(gz);
OffsetX=-MeanGX;
OffsetY=-MeanGY;
OffsetZ=-MeanGZ;
set(handles.ogx,'String',OffsetX);
set(handles.ogy,'String',OffsetY);
set(handles.ogz,'String',OffsetZ);

grid(handles.axes1,'on');
plot(handles.axes1,gx+OffsetX,'r');
hold(handles.axes1,'on');
plot(handles.axes1,gy+OffsetY,'g');
plot(handles.axes1,gz+OffsetZ,'b');
xlabel(handles.axes1, 'Muestras');
ylabel(handles.axes1,'Giroscopio Calibrado [rad/s]');
hold(handles.axes1,'off');

% --- Executes on button press in conectar.
function conectar_Callback(hObject, eventdata, handles)
% hObject    handle to conectar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global state s muestras
if state==false
    delete(instrfind({'Port'},{'COM7'}));
    s = serial('COM7','BaudRate',921600,'Terminator','CR/LF');
    warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
    fopen(s);
    fwrite(s,';','char');
    pause(3);
    set(handles.conectar,'BackgroundColor','green'); 
    set(handles.conectar,'String','Desconectar');
    state= true;
else
    fclose(s);
    delete(s);
    clear s;
    set(handles.conectar,'BackgroundColor','blue'); 
    set(handles.conectar,'String','Conectar');
    state= false;
end


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);


% --------------------------------------------------------------------
function Untitled_1_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1
global numSensor
numSensor=get(hObject,'Value') - 1;


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
