function  Mobile_Graph=CarlikePlot(dx,dy,angz,psi)
global  Carlike;

%Matriz de rotación z

Rz=[ cos(angz) -sin(angz) 0; sin(angz) cos(angz) 0; 0 0 1];

Rz1=[ cos(angz+psi) -sin(angz+psi) 0; sin(angz+psi) cos(angz+psi) 0; 0 0 1];
dx1=0.75*cos(angz);
dy1=0.75*sin(angz);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
robotPatch = Rz*Carlike.platformVertices;
robotPatch(1,:)=robotPatch(1,:)+dx;
robotPatch(2,:)=robotPatch(2,:)+dy;

Mobile_Graph(1) = patch('Faces',Carlike.platformFaces,'Vertices',robotPatch','FaceColor',   [0.3 0.75 0.93],'EdgeColor','none');

robotPatch = Rz*Carlike.platformfVertices;
robotPatch(1,:)=robotPatch(1,:)+dx;
robotPatch(2,:)=robotPatch(2,:)+dy;

Mobile_Graph(2) = patch('Faces',Carlike.platformfFaces,'Vertices',robotPatch','FaceColor',[0 0.75 0.75],'EdgeColor','none');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robotPatch = Rz* Carlike.wheelVertices;
robotPatch(1,:)=robotPatch(1,:)+dx;
robotPatch(2,:)=robotPatch(2,:)+dy;

Mobile_Graph(3) = patch('Faces',Carlike.wheelFaces,'Vertices',robotPatch','FaceColor',[0.8 0.8 0.8],'EdgeColor','none');

robotPatch = Rz1*Carlike.wheel1Vertices;
robotPatch(1,:)=robotPatch(1,:)+dx+dx1;
robotPatch(2,:)=robotPatch(2,:)+dy+dy1;

Mobile_Graph(4) = patch('Faces',Carlike.wheel1Faces,'Vertices',robotPatch','FaceColor',[0.8 0.8 0.8],'EdgeColor','none');







