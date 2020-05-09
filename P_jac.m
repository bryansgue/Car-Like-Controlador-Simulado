clc
clear all
close all

x1=1;   % posición inicial (eje x) en m
y1=1;   % posición inicial (eje y) en m
phi=1;  % orientación inicial ruedas traseras en rad
psi=2; 
D=4;
% syms phi psi D u w
    %b) Matriz Jacobiana
    J=[cos(psi) 0
        sin(psi) 0
        (1/D)*sin(psi-phi) 0
        0 1]
    trans=J'
    
%     J1 = inv(J*J')*J'
J1 = (J*J')
    p=inv(J1)
    r= p*J
    

% %     v1 = (v*v')
%     
%     x = J1*v
% %     
%      xpp  = x(1)
%      ypp  = x(2)
% %      phip = x(3)
%      psip = x(4)
    