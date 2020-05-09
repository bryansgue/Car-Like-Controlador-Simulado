function [VMref] = vref_carlike (xd,yd,phid,xd_p,yd_p,phid_p,hx,hy,phi)

% J=[cos(phi)-tan(psi)*sin(phi) 0
%    sin(phi)+tan(psi)*cos(phi) 0
%    (1/D)*tan(psi) 0
%    0 1];

J=[0 1
   cos(phi) 0
   sin(phi) 0];


%3) Calculos del Error
  phie= Angulo(phid - phi); 
  hxe= xd - hx;
  hye= yd - hy;
  %psie= (psid-psi);   
  he= [phie hxe hye ];
  
  K1 = diag(1*[2 0.75 1]);
  K2 = diag(0.5*[1 0.5 3]); 

  VMref = pinv(J)*([phid_p xd_p yd_p ]'+K1*tanh(K2*he'))
  %VMref = pinv(J)*(K1*tanh(K2*he'))

end