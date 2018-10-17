%% ArduinoSerialMatlab - github.com/jdvr1994
%% Copyright Juan David Velasquez 2018
%% MIT License

%% CONFIGURACION PUERTO COM
% Eliges el puerto COM a utilizar (en este ejemplo COM4) 
% a una velocidad (en este ejemplo 19200) OJO DEBE SER IGUAL A LA
% CONFIGURADA EN ARDUINO en Serial.begin(19200)
% 'Terminator','CR/CL' define caracter que le indicara que termina una
% trama de datos, en este caso 'CR/CL' === SALTO DE LINEA
% En el codigo de arduino despues de enviar
% dataX,dataY,dataZ se envia un (\n) al usar el metodo println

delete(instrfind({'Port'},{'COM21'}));
s = serial('COM21','BaudRate',19200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
fopen(s);
fwrite(s,';','char');
pause(3);

%% Recuperamos datos desde el ARDUINO
clear ax ay az gx gy gz
NUM_MUESTRAS = 500;

% Configuracion graficas
figure('rend','painters','pos',[50 50 900 600])
subplot(2,3,1);
axPlot = plot(0,'Color','r');
title('Acelerometro X');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

subplot(2,3,2);
ayPlot = plot(0,'Color','g');
title('Acelerometro Y');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

subplot(2,3,3);
azPlot = plot(0,'Color','b');
title('Acelerometro Z');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

subplot(2,3,4);
gxPlot = plot(0,'Color','r');
title('Giroscopo X');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

subplot(2,3,5);
gyPlot = plot(0,'Color','g');
title('Giroscopo Y');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

subplot(2,3,6);
gzPlot = plot(0,'Color','b');
title('Giroscopo Z');
axis([0 NUM_MUESTRAS -32768 32767]);
axis('manual');
grid on;

pause(0.2);

% Captura de muestras desde Arduino
for i=1:NUM_MUESTRAS
    %Envio 'a' para solicitar los datos del accelerometro
    fwrite(s,'a','char');
    acelerometro = fscanf(s,'%f,%f,%f')';
    ax(i)=acelerometro(1);
    ay(i)=acelerometro(2);
    az(i)=acelerometro(3);
    
    %Envio 'g' para solicitar los datos del accelerometro
    fwrite(s,'g','char');
    giroscopos = fscanf(s,'%f,%f,%f')';
    gx(i)=giroscopos(1);
    gy(i)=giroscopos(2);
    gz(i)=giroscopos(3);
    
    set(axPlot,'ydata',ax);
    set(ayPlot,'ydata',ay);
    set(azPlot,'ydata',az);
    set(gxPlot,'ydata',gx);
    set(gyPlot,'ydata',gy);
    set(gzPlot,'ydata',gz);
    
    pause(0.0001);
end
%% Cerramos el puerto Serial
fclose(s);
delete(s);
clear s;
