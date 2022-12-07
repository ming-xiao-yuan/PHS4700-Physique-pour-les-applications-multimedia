% Fonction qui permet de savoir s'il y a une collision ou pas

function calculCollision = calculCollision(q0, sommetMin, count)

   # Puisque le sol est immobile, il n'est pas necaisse de calculer ses parametres

  vDinit = [q0(4); q0(5); q0(6)];        # Vitesse lineaire du de de juste avant collision
  wDinit = [q0(7); q0(8); q0(9)];        # Vitesse angulaire du de juste avant collision
  n = [0; 0; 1];

  longueurCote = 0.04;                # En metre
  masseDe = 0.02;                     # En kg
  muS = 0.5;
  coefRestitution = 0.5;
  muC = 0.3;
  inertieDe = (masseDe * longueurCote^2)/6 * [1, 0, 0; 0, 1, 0; 0, 0, 1];
  rAP = [q0(1); q0(2); q0(3)] - sommetMin;

  vDMoins = vDinit + cross(wDinit, rAP);

  u = cross(vDMoins, n)/norm(cross(vDMoins, n));
  t = cross(n, u);

  vDScalaireMoins = dot(n, vDMoins);

  termeGauche = muS * (1 + coefRestitution) * abs(vDScalaireMoins);
  termeDroite = abs(dot(t, vDMoins));

  x = cross((inv(inertieDe) * cross(rAP, t)), rAP);

  GaT = dot(t,x);

  # On peut ignorer le sol puisque 1/mSol = 0 et rBP cross n = 0

  acceleration = 1/((1/masseDe) + GaT);

  jt = 0;

  if(termeGauche < termeDroite)
    jt = acceleration * muC * (1 + coefRestitution) * vDScalaireMoins;
  else
    jt = -acceleration * abs(dot(t, vDMoins));
  endif

  if(count > 5)
    jt = 0;
  endif

  j = -acceleration * (1 + coefRestitution) * vDScalaireMoins;

  jVec = n * j + t * jt;

  vDFinal = vDinit + jVec/masseDe;

  wDFinal = wDinit + inv(inertieDe) * cross(rAP, jVec);

  calculCollision = [vDFinal, wDFinal];
end
