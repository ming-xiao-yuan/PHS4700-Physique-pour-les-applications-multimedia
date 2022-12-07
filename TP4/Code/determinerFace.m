function [distanceF,posFinale,faceF]=determinerFace(distanceRayon,distance,position,direction,posDepart,face,couleur)
posFinale=position;
faceF=face;
distanceF=distance+distanceRayon;
posFinale{end+1}=distanceF*direction+posDepart;
faceF{end+1}=couleur;
end