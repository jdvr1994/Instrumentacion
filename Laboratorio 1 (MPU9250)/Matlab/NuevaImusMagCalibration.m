%% Calibracion Magnetometro Hard Iron
%Descartamos las primeras medidas erroneas
a=10;
load('variablesMag.mat');
%Definimos los vectores de medición
x=x(a:end);
y=y(a:end);
z=z(a:end);

plot3(x,y,z);

x2=x.^2;
y2=-y.^2;
z2=-z.^2;

%Ecuacion W=H*X ==>> X=([Ht*H]^-1)*W
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

figure(2);
plot3(xxx,yyy,zzz);
axis equal;

disp(Xoffset-1000);
disp(Yoffset-1000);
disp(Zoffset-1000);
disp(XScale);
disp(YScale);
disp(ZScale);
