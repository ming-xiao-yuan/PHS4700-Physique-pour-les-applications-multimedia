% Devoir1 fonction
function [pcm, acm, MI, aa]=Devoir1(pos,ar,va,Lambda)
cmBoite=[0;0.1;-0.15]; %Centre de masse colis
ag=[0;0;-9.81]; %Acceleration gravitationnelle (m/s2)
rs=0.3; %Rayon demi-sphsere (m)
ms=1.5; %Masse demi-sphere (kg)
lb=0.5; %Longueur bras (m)
mb=0.2; %Masse bras (m)
rb=0.025; %Rayon bras (m)
Hm=0.1; %Hauteur moteur (m)
rm=0.05; %Rayon moteur (m)
mm=0.4; %Masse moteur (kg)
normeFmax=20; %Force moteur (N)
mc=0.8; %Masse colis (kg)
Lc=0.71; %Longueur colis (m)
lc=0.4; %Largeur colis (m)
Hc=0.3; %Hauteur colis (m)
mDrone=ms+6*mb+6*mm;%Masse drone(kg)
mTotale=mc+mDrone; %Masse totale drone + colis(kg)
matriceRotationY=[cos(ar) 0 sin(ar); 0 1 0;-sin(ar) 0 cos(ar)]; %Matrice de rotation en OY
matriceRotationZ60Degree=[cosd(60) -sind(60) 0;sind(60) cosd(60) 0;0 0 1]; %Matrice de rotation OZ avec 60% pour inertie


%Calcul centre de masse demi-sphere
cmDemiSphere=[0;0;(3*rs)/8];

%Calcul centre de masse bras
cmB1=[rs+(lb/2);0;rb];
cmB2=[(rs+lb/2)*cosd(60);(rs+lb/2)*sind(60);rb];
cmB3=[-((rs+lb/2)*cosd(60));(rs+lb/2)*sind(60);rb];
cmB4=[-(rs+(lb/2));0;rb];
cmB5=[-((rs+lb/2)*cosd(60));-((rs+lb/2)*sind(60));rb];
cmB6=[(rs+lb/2)*cosd(60);-((rs+lb/2)*sind(60));rb];

%Calcul centre de masse moteurs
cmM1=[rs+lb+rm;0;Hm/2];
cmM2=[(rs+lb+rm)*cosd(60);(rs+lb+rm)*sind(60);Hm/2];
cmM3=[-((rs+lb+rm)*cosd(60));(rs+lb+rm)*sind(60);Hm/2];
cmM4=[-(rs+lb+rm);0;Hm/2];
cmM5=[-((rs+lb+rm)*cosd(60));-((rs+lb+rm)*sind(60));Hm/2];
cmM6=[(rs+lb+rm)*cosd(60);-((rs+lb+rm)*sind(60));Hm/2];
cm6Moteurs=(mm*(cmM1+cmM2+cmM3+cmM4+cmM5+cmM6))/(6*mm);

%Calcul centre de masse bras-moteur
cmbm=(mm*(cmM1+cmM2+cmM3+cmM4+cmM5+cmM6)+mb*(cmB1+cmB2+cmB3+cmB4+cmB5+cmB6))/(mm+mb);

%Calcul centre de masse du drone
cmTotal=(cmbm*(mm+mb)+ms*cmDemiSphere+mc*cmBoite)/(ms+mm+mb+mc);

%Calcul centre de masse avec ar
pcm=pos+(matriceRotationY*cmTotal);

%Creation des forces moteurs
F1=matriceRotationY*(Lambda(1)*normeFmax*[0;0;1]);
F2=matriceRotationY*(Lambda(2)*normeFmax*[0;0;1]);
F3=matriceRotationY*(Lambda(3)*normeFmax*[0;0;1]);
F4=matriceRotationY*(Lambda(4)*normeFmax*[0;0;1]);
F5=matriceRotationY*(Lambda(5)*normeFmax*[0;0;1]);
F6=matriceRotationY*(Lambda(6)*normeFmax*[0;0;1]);

%Acceleration centre de masse
FgDrone=mDrone*ag;
FgColis=mc*ag;

sommeForces=[F1 F2 F3 F4 F5 F6 FgDrone FgColis];
acm=sum(sommeForces,2)/mTotale;

%Calcul moment inertie demi-sphere par rapport a centre de masse drone
icDemiSphere=(ms*rs*rs)*[83/320 0 0;0 83/320 0;0 0 2/5];
dcDemiSphere=(pos-pcm)-cmDemiSphere;
idDemisSphere=icDemiSphere+ms*T(dcDemiSphere);

%Calcul moment inertie bras par rapport a centre de masse drone
icB1=[mb*rb*rb 0 0;0 (mb*rb*rb)/2+(mb*lb*lb)/12 0; 0 0 (mb*rb*rb)/2+(mb*lb*lb)/12];
dcB1=(pos-pcm)-cmB1; %Trouver vecteur dc
idB1=icB1+mb*T(dcB1);%Moment inertie bras 1 par rapport au CM drone+colis
icB2=matriceRotationZ60Degree*icB1;
dcB2=(pos-pcm)-cmB2;
idB2=icB2+mb*T(dcB2);
icB3=matriceRotationZ60Degree*icB2;
dcB3=(pos-pcm)-cmB3;
idB3=icB3+mb*T(dcB3);
icB4=matriceRotationZ60Degree*icB3;
dcB4=(pos-pcm)-cmB4;
idB4=icB4+mb*T(dcB4);
icB5=matriceRotationZ60Degree*icB4;
dcB5=(pos-pcm)-cmB5;
idB5=icB5+mb*T(dcB5);
icB6=matriceRotationZ60Degree*icB5;
dcB6=(pos-pcm)-cmB6;
idB6=icB6+mb*T(dcB6);

idBras=idB1+idB2+idB3+idB4+idB5+idB6;%Moment d'inertie totale des 6 bras par rapport au CM drone+colis

%Calcul moment inertie moteur par rapport a centre de masse drone
icM1=[(mb*rb*rb)/4+(mb*lb*lb)/12 0 0;0 (mb*rb*rb)/4+(mb*lb*lb)/12 0; 0 0 (mb*rb*rb)/2];
dcM1=(pos-pcm)-cmM1;
idM1=icM1+mm*T(dcM1);
icM2=icM1;
dcM2=(pos-pcm)-cmM2;
idM2=icM2+mm*T(dcM2);
icM3=icM1;
dcM3=(pos-pcm)-cmM3;
idM3=icM3+mm*T(dcM3);
icM4=icM1;
dcM4=(pos-pcm)-cmM4;
idM4=icM4+mm*T(dcM4);
icM5=icM1;
dcM5=(pos-pcm)-cmM5;
idM5=icM5+mm*T(dcM5);
icM6=icM1;
dcM6=(pos-pcm)-cmM6;
idM6=icM6+mm*T(dcM6);

idMoteur=idM1+idM2+idM3+idM4+idM5+idM6;

%Calcul moment inertie boite par rapport a centre de masse drone x->z y->y z->x
icBoite=[mc/12*((lc*lc)+(Hc*Hc)) 0 0;0 mc/12*((Hc*Hc)+(Lc*Lc)) 0;0 0 mc/12*((lc*lc)+(Lc*Lc))];
dcBoite=(pos-pcm)-cmBoite;
idBoite=icBoite+mb*T(dcBoite);

%Calcul inertie totale par rapport au cm drone
MI=idDemisSphere+idBras+idMoteur+idBoite;

%Calcul vitesse angulaire
sommeForcesMoteurs=[F1 F2 F3 F4 F5 F6];
L=MI*va;
To=matriceRotationY*(cross((cm6Moteurs-(pos-pcm)),sum(sommeForcesMoteurs,2)));
aa=inv(MI)*(To+cross(L,va));
end



