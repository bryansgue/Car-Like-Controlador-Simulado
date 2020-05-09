clc  % Borrar ventana de comandos
clear %Eliminar elementos del espacio de trabajo, liberando memoria del sistema
close all % Eliminar figura especificada

ts=0.1; % tiempo de muestra
t=0:ts:50; %tiempo de vector

% 2) Condiciones inicialesWEerweweweewewe
    D=0.75;

    x1(1)=-2;   % posición inicial (eje x) en m
    y1(1)=2;   % posición inicial (eje y) en m
    phi(1)=-pi/8;  % orientación inicial ruedas traseras en rad
    psi(1)=pi/6;  % orientación inicial reudas delanteras en rad

    hx(1)=x1(1)+D*cos(phi(1));   % posición inicial (eje x) en metros
    hy(1)=y1(1)+D*sin(phi(1));   % posición inicial (eje y) en m


% 3) Referencias deseadas
    %Trayectoria de un circulo
%     xrd = 4*cos(0.2*t);
%     yrd = 4*sin(0.2*t)+1;
    
    %linea recta
%     xrd= t;
%     yrd= 0*t;
    
%     xrdp= -4*0.2*sin(0.2*t);  %Posición x1
%     yrdp= 4*0.2*cos(0.2*t);    %Posición y1
    
    %Trayectoria senoidal
    hxd = 0.1*t
    hyd = 2*cos(0.1*t)
%     %Trayectoria senoidal
%     hxd = 0.8*t;
%     hyd = cos(0.8*t);
    hxdp= diff([0 hxd]);  %Posición x1
    hydp= diff([0 hyd]);
    
%     phid= hydp/0.1;
%     psid= hydp/0.1;
    
    %Trayectoria de un 8
%     xrd= 4*sin(0.4*t);
%     yrd=4*sin(0.2*t);
%     
    %Trayectoria de un corazon 
%     xrd= (12*sin(0.1*t)-4*sin(3*0.1*t))/4;
%     yrd= (13*cos(0.1*t)-5*cos(2*0.1*t)-2*cos(3*0.1*t)-cos(4*0.1*t))/4;
    hxdp= diff([0 hxd]);  %Posición x1
    hydp= diff([0 hyd]);  %Posición y1
%     phidp = diff([0 phid]);
%     psidp = diff([0 psid]);

% u=0.2*ones(1,length(t));
% % wsi=(15*(pi/180))*sin(0.01*t);
% 
% % psi=5*ones(1,length(t));
% psi= 10*cos(t); % angulo ruedas delanteras
% wsi=diff([0 psi]); %velocidad angular ruedas delanteras

for k=1:length(t)
    
    %a) Errores de control
    hxe(k) = hxd(k) - hx(k);
    hye(k) = hyd(k) - hy(k);
%     phie(k) = phid(k) - phi(k);
%     psie(k) = psid(k) - psi(k);
%     e= [hxe(k);hye(k); phie(k); psie(k)]
    e= [hxe(k);hye(k)]

%     hxp(k)=u(k)*cos(phi(k))-u(k)*tan(psi(k))*sin(phi(k));
%     hyp(k)=u(k)*sin(phi(k))+u(k)*tan(psi(k))*cos(phi(k));
%     phip(k)=(1/D)*u(k)*tan(psi(k));
%     psip(k)=wsi(k);
    
    %b) Matriz Jacobiana
    J=[cos(phi(k))-tan(psi(k))*sin(phi(k)) 0
       sin(phi(k))+tan(psi(k))*cos(phi(k)) 0
       (1/D)*tan(psi(k)) 0
       0 1];
    a2 = J;
    at = J'
    J1 = J*at
    J1_inv = inv(J1)
%     J_trans= J*J'ç

    J1_f = J1_inv*J
%     J1_f = J1_f'
%     J1_f_trans = J1_f'

    %c) Matrix de ganancia
%     K= [1 0 0 0 
%         0 1 0 0
%         0 0 1 0
%         0 0 0 1];
    K= [1 0 
        0 1];

    %d) Ley de control
%         hdp=[xrdp(k);yrdp(k)];
%         hdp=[hxdp(k); hxdp(k); phidp(k); psidp(k)]
        hdp=[hxdp(k); hydp(k)]
%         K_result=K*e
%         hdp_resul=(hdp+K*e)
        result=hdp+K*e
        v= J1_f*(hdp+K*e)
%         v= inv(J)*(hdp);
     u(k)= v(1);
     w(k)= v(2);

    % Modelo cinemático
%     hxp(k)=u(k)*cos(phi(k)+psi(k));
%     hyp(k)=u(k)*sin(phi(k)+psi(k));
%     phip(k)=(1/D)*u(k)*sin((phi(k)+psi(k))-phi(k));
%     psip(k)=w(k);
    
    hxp(k)=u(k)*cos(phi(k))-u(k)*tan(psi(k))*sin(phi(k));
    hyp(k)=u(k)*sin(phi(k))+u(k)*tan(psi(k))*cos(phi(k));
    phip(k)=(1/D)*u(k)*tan(psi(k));
    psip(k)=w(k);

    %integral numérica (método de Euler)
    hx(k+1)=hx(k)+ts*hxp(k);
    hy(k+1)=hy(k)+ts*hyp(k);
    phi(k+1)=phi(k)+ts*phip(k);
    psi(k+1)=psi(k)+ts*psip(k);

    x1(k+1)=hx(k+1)-D*cos(phi(k+1));   % posición inicial (eje x) en m
    y1(k+1)=hy(k+1)-D*sin(phi(k+1));   % posición inicial (eje y) en m


end

scene=figure;  % nueva figura
tam=get(0,'ScreenSize');
set(scene,'position',[tam(1) tam(2) tam(3) tam(4)]); % posición y tamaño figura en la pantalla

axis equal; % Establecer relaciones de aspecto del eje
axis([-6 6 -6 6 -0.1 1]); % Establecer límites de eje 
grid on; % Mostrar líneas de cuadrícula de ejes
MobileCarlike; % Parámetros del robot
M1=CarlikePlot(x1(1),y1(1),phi(1),psi(1)); % Trazar el robot en posición inicial hx, hy y phi orientación
hold on; % Conservar la trama actual al agregar una nueva trama
M2=plot(hx(1),hy(2),'r'); %Trayectoria de la trama.
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Eje de etiqueta
camlight('rigth');
hold on, %plot(xrd,yrd,'r');
plot(hxd,hyd,'b');

% 
step=30; % paso de posición
pause(2);
for i=1:step:length(t) % Emulación de bucle
    delete (M1)
    delete (M2)
    M1=CarlikePlot(x1(i),y1(i),phi(i),psi(i)); hold on
    plot(hx(1:i),hy(1:i),'b','LineWidth',2); % Trayectoria de la trama.
    pause(ts)
end


