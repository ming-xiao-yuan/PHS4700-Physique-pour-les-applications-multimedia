# Fonction qui calcule la force de frottement

function calculForceFrottement = calculForceFrottement(q0, masse, temps)

  muC = 0.5;
  N = masse * 9.81;
  vGlissementInit = [q0(4); q0(5); q0(6)]
  rInit = [q0(1); q0(2); q0(3)];

  fF = -muC * N * vGlissementInit/norm(vGlissementInit)

  acceleration = fF/masse

  vGlissementFinal = vGlissementInit + acceleration * temps;

  rFinal = rInit + vGlissementInit * temps + 0.5 * acceleration * temps^2;

  calculForceFrottement = [rFinal vGlissementFinal];


  end
