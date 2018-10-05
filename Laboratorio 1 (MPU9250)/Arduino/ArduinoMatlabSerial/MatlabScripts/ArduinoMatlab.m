%% ArduinoSerialMatlab - github.com/jdvr1994
%% Copyright Juan David Velasquez 2018
%% MIT License

%% CONFIGURACION PUERTO COM
% Eliges el puerto COM a utilizar (en este ejemplo COM4) 
% a una velocidad (en este ejemplo 19200) OJO DEBE SER IGUAL A LA
% CONFIGURADA EN ARDUINO en Serial.begin(19200)
% Los otros parametros de configuracion:::
% 'Terminator','CR/CL' define caracter que le indicara que termina una
% trama de datos, en este caso 'CR/CL' === SALTO DE LINEA
% En el codigo de arduino te daras cuenta que despues de enviar
% dataX,dataY,dataZ se envia un (\n) al usar el metodo println

delete(instrfind({'Port'},{'COM4'}));
s = serial('COM4','BaudRate',19200,'Terminator','CR/LF');
warning('off','MATLAB:serial:fscanf:unsuccessfulRead');
fopen(s);
fwrite(s,';','char');
pause(3);

%% Recuperamos datos desde el ARDUINO
NUM_MUESTRAS = 10;
for i=1:NUM_MUESTRAS
    %Envio 'a' para solicitar los datos del accelerometro
    fwrite(s,'a','char');
    acelerometro = fscanf(s,'%d,%d,%d')';
    ax(i)=acelerometro(1);
    ay(i)=acelerometro(2);
    az(i)=acelerometro(3);
    
    %Envio 'g' para solicitar los datos del accelerometro
    fwrite(s,'g','char');
    giroscopos = fscanf(s,'%d,%d,%d')';
    gx(i)=giroscopos(1);
    gy(i)=giroscopos(2);
    gz(i)=giroscopos(3);
end

%% Graficamos los datos
figure
plot(ax);
hold on;
plot(ay);
hold on;
plot(az);

figure
plot(gx);
hold on;
plot(gy);
hold on;
plot(gz);


%% Cerramos el puerto Serial
fclose(s);
delete(s);
clear s;
