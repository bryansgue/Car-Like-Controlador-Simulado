clc  % Borrar ventana de comandos
clear %Eliminar elementos del espacio de trabajo, liberando memoria del sistema
close all % Eliminar figura especificada

ts=0.1; % tiempo de muestra
t=0:ts:180; %tiempo de vector


%u=0.1*ones(1,length(t));

%psi= 5*cos(0.1*t); % angulo ruedas delanteras
%w_psi=diff([0 psi]); %velocidad angular ruedas delanteras


D=0.75;

x1(1)=0;   % posición inicial (eje x) en m
y1(1)=0;   % posición inicial (eje y) en m
phi(1)=0;  % orientación inicial ruedas traseras en rad
psi(1)=0;  % orientación inicial reudas delanteras en rad

hx(1)=0;   % posición inicial (eje x) en metros
hy(1)=0;   % posición inicial (eje y) en m

%Trayectoria SENO
xd= 0.1*t ;                        
xd_p= 0.1* ones(1,length(t));     
xd_pp= 0* ones(1,length(t));

yd= 0.5 * sin(0.1*t);         
yd_p= 0.5*0.1 * cos(0.1*t);         
yd_pp= -0.5* 0.1* 0.1* sin(0.1*t);

%%Trayectoria OCHO
%  xd = 10 * sin(0.04*t)+0.1;
%  xd_p = 10*0.04*cos(0.04*t);     
%  xd_pp = -10*0.04*0.04*sin(0.04*t);
%  yd = 5 * sin(0.08*t)+0.1;         
%  yd_p = 5*0.08*cos(0.08*t);     
%  yd_pp = -5*0.08*0.08*sin(0.08*t);               
         
   % d) Trayectoria Silla de Montar
% 3) CÃ¡lculo de orientaciÃ³n 
phid= (atan2(yd_p,xd_p));
phid_p = (1./((yd_p./xd_p).^2+1)).*((yd_pp.*xd_p-yd_p.*xd_pp)./xd_p.^2);
%phid_p(1)=0; 

for k=1:length(t)
    %psid(k)=0.01;
    %psid_p(k)=0.0001;
    
    respuesta = vref_carlike(xd(k),yd(k),phid(k),xd_p(k),yd_p(k),phid_p(k),hx(k),hy(k),phi(k));
    u(k) = respuesta(1);
    w(k) = respuesta(2);
    
    % Modelo cinemático      xd,   yd,    phid,   psid,  xd_p,   yd_p,              psid_p,  hx,    hy,   phi,   psi,  D
    phi_p(k)= w(k);
    hx_p(k)=u(k)*cos(phi(k));
    hy_p(k)=u(k)*sin(phi(k));

    %integral numérica (método de Euler)
    phi(k+1) = Angulo(w(k)*ts + phi(k));
    hx(k+1) = hx_p(k)*ts + hx(k);
    hy(k+1) = hy_p(k)*ts + hy(k);
    
    psi(k+1) = atan((D*w(k))/u(k));
          
    %Graficas
    %plot3(xu(1:k),yu(1:k),zu(1:k),'r')
    %hold on
end

scene=figure;  % nueva figura
tam=get(0,'ScreenSize');
set(scene,'position',[tam(1) tam(2) tam(3) tam(4)]); % posición y tamaño figura en la pantalla

axis equal; % Establecer relaciones de aspecto del eje
%axis([-4 10 -6 6 -0.1 1]); % Establecer límites de eje 
grid on; % Mostrar líneas de cuadrícula de ejes
MobileCarlike; % Parámetros del robot
M1=CarlikePlot(x1(1),y1(1),phi(1),psi(1)); % Trazar el robot en posición inicial hx, hy y phi orientación
hold on; % Conservar la trama actual al agregar una nueva trama
%M2=plot(hx(1),hy(2),'r'); %Trayectoria de la trama.
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Eje de etiqueta
camlight('rigth');

step=10; % paso de posición

% GRAF DEL CARRO
  for i=1:step:length(t) % Emulación de bucle
    delete (M1)
    %delete (M2)
    M1=CarlikePlot(hx(i),hy(i),phi(i),psi(i)); 
    hold on
    plot(xd(1:i),yd(1:i),'b')
    hold on
    plot(hx(1:i),hy(1:i),'r','LineWidth',1); % Trayectoria de la trama.
    pause(ts)
end

% 1) Igualar columnas de los vectores creados
  hx(:,end)=[];
  hy(:,end)=[];
  phi(:,end)=[];
  
% 2) CÃ¡lculos del Error
  figure(2)
  hxe= xd - hx;
  hye= yd - hy;
  phie= Angulo(phid-phi);
  plot(hxe), hold on, grid on
  plot(hye)
  plot(phie)
  legend("hxe","hye","phie")
  title ("Errores de posiciÃ³n")
  
% 3) Posiciones deseadas vs posiciones reales del extremo operativo del manipulador aÃ©reo
  figure(3)
  
  subplot(3,1,1)
  plot(xd)
  hold on
  plot(hx)
  legend("xd","hx")
  ylabel('x [m]'); xlabel('s [ms]');
  title ("Posiciones deseadas y reales del extremo operativo del manipulador aÃ©reo")
  
  subplot(3,1,2)
  plot(yd)
  hold on
  plot(hy)
  legend("yd","hy")
  ylabel('y [m]'); xlabel('s [ms]');

  subplot(3,1,3)
  plot(Angulo(phid))
  hold on
  plot(phi)
  legend("psid","psi")
  ylabel('psi [rad]'); xlabel('s [ms]');
  
  

