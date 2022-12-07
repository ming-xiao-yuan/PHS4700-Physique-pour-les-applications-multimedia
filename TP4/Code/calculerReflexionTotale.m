function estReflexion=calculerReflexionTotale(nt,ni,n,direction)
if (acos(dot(n,direction))<asin(nt/ni))&&ni>nt
    estReflexion=true;
else
    estReflexion=false;
end
end