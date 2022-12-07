function [sortie]=T(entree)
dcx=entree(1);
dcy=entree(2);
dcz=entree(3);

sortie=[(dcy*dcy)+(dcz*dcz) -dcx*dcy -dcx*dcz;-dcy*dcx (dcx*dcx)+(dcz*dcz) -dcy*dcz;-dcz*dcx -dcz*dcy (dcx*dcx)+(dcy*dcy)];
end