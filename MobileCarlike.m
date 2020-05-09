function  MobileCarlike
load('CarlikeDimensions.mat')
global Carlike;

% 1 m de largo.
% 0.5 m de ancho
Carlike.platformVertices=PlatformCarlike.vertices';
Carlike.platformFaces=PlatformCarlike.faces;

Carlike.platformfVertices=PlatformFront.vertices';
Carlike.platformfFaces=PlatformFront.faces;

%0.15 m radio
Carlike.wheelVertices=wheelCarlike.vertices';
Carlike.wheelFaces=wheelCarlike.faces;

Carlike.wheel1Vertices=wheelCarlike1.vertices';
Carlike.wheel1Faces=wheelCarlike1.faces;



