%% Calibracion Soft Iron Magnetometro FINAL

a=10;
muestras=size(mx,2)
%Obtencion de parametros de calibracion
x=mx(a:muestras);
y=my(a:muestras);
z=mz(a:muestras);
x2=x.^2;
y2=y.^2;
z2=z.^2;
xy=x.*y;
xz=x.*z;
yz=y.*z;

D=[x2' y2' z2' 2*xy' 2*xz' 2*yz' 2*x' 2*y' 2*z' ];
v=inv(D'*D)*D'*ones(size(x,2),1);
Vghi=v(7:9);
A4=[v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -1];
A3=[v(1) v(4) v(5); v(4) v(2) v(6); v(5) v(6) v(3)];

Offsets= -inv(A3)*Vghi;

T=[1 0 0 0; 0 1 0 0; 0 0 1 0; Offsets(1) Offsets(2) Offsets(3) 1];
B4=T*A4*T';
B3=B4(1:3,1:3)/-B4(4,4);
[rotM ev]=eig(B3);
gain=sqrt(1./diag(ev));

%Calibracion de la señal
XC=x-Offsets(1);
YC=y-Offsets(2);
ZC=z-Offsets(3);

XYZC=[XC' YC' ZC']*rotM;
XC=XYZC(:,1)/gain(1);
YC=XYZC(:,2)/gain(2);
ZC=XYZC(:,3)/gain(3);

subplot(2,2,1); hold on; plot(XC,YC,'ro'); plot(x,y,'kx');
xlabel('X'); ylabel('Y'); axis equal; grid on;
subplot(2,2,2); hold on; plot(ZC,YC,'go'); plot(z,y,'kx');
xlabel('Z'); ylabel('Y'); axis equal; grid on;
subplot(2,2,3); hold on; plot(XC,ZC,'bo'); plot(x,z,'kx');
xlabel('X'); ylabel('Z'); axis equal; grid on;

Magnitud=sqrt(XC.^2 + YC.^2 + ZC.^2);