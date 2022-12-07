% Fonction qui roule le devoir 3

function [face t x y z sommets] = Devoir3(Pos0, MatR0, V0, W0)

    face = 1;                 # Face superieure du de
    t = 0;                    # En secondes
    deltaTemps = 0.01;           # Difference de temps entre chaque iteration
    longueurCote = 0.04;      # En m
    masseDe = 0.02;            # En kg
    accelerationG = [0; 0; -9.81];
    hauteur = [0; 0; 0];

    # Position de depart      # En m

    x = Pos0(1);
    y = Pos0(2);
    z = Pos0(3);

    # Vitesse de depart       # m/s
    vX = V0(1);
    vY = V0(2);
    vZ = V0(3);

    #Vitesse Angulaire

    wX = W0(1);
    wY = W0(2);
    wZ = W0(3);

    #Matrice de rotation
    Rxx = MatR0(1);
    Rxy = MatR0(2);
    Rxz = MatR0 (3);

    Ryx = MatR0(4);
    Ryy = MatR0(5);
    Ryz = MatR0(6);

    Rzx = MatR0(7);
    Rzy = MatR0(8);
    Rzz = MatR0(9);

    q = [x y z vX vY vZ wX wY wZ Rxx Rxy Rxz Ryx Ryy Ryz Rzx Rzy Rzz];

    indice = 2;

    rayonSphereDe = (longueurCote * sqrt(3))/2;
    count = 0;
    finSimulation = false;

    while true

      t = [t t(indice-1) + deltaTemps];


      q0 = q;

      g = @calculParamDeAvCollision;

      q = SEDRKT4t0(q0, t(indice-1), deltaTemps, g);

      x = [x q(1)];
      y = [y q(2)];
      z = [z q(3)];

      vX = [vX q(4)];
      vY = [vY q(5)];
      vZ = [vZ q(6)];
      vitesseZ = q(6);

      conditionCollision = z(indice) - rayonSphereDe;

      vitesseLinDe = [q(4); q(5); q(6)];
      vitesseAngDe = [q(7); q(8); q(9)];
      inertieDe = (masseDe * longueurCote^2)/6;

      hauteur = [0; 0; q(3)];




      if(conditionCollision  <= 0.001 && conditionCollision >= -0.001 && vitesseLinDe(3) < 0)

          sommets = calculPositionAngulaire(q, indice);
          sommetMin = calculSommetCollision(sommets);
          result = calculCollision(q, sommetMin, count);
          vFDe = [result(1); result(2); result(3)];
          wFDe = [result(4); result(5); result(6)];

          q(4) = abs(vFDe(1));
          q(5) = abs(vFDe(2));
          q(6) = abs(vFDe(3));



          q(7) = wFDe(1);
          q(8) = wFDe(2);
          q(9) = wFDe(3);

          count = count + 1;

          if(finSimulation == true)
            break;
          endif
          if(count > 100)
            q(6) = 0;
            temps = 0;
            while (q(4) > 0 && q(5) > 0)
              temps = temps + deltaTemps
              position = calculForceFrottement(q, masseDe, temps);

              x = [x position(1)];
              y = [y position(2)];
              z = [z position(3)];

              q(4) = position(4);
              q(5) = position(5);
              q(6) = position(6);


              vitesseLinFinal = [q(4); q(5); q(6)]
              vitesseAngFinal = [q(7); q(8); 0]
            endwhile
            if(vitesseLinFinal(1) < 0 && vitesseLinFinal(2) < 0)

               conditionArret = 0.5 * masseDe * dot(vitesseLinFinal,vitesseLinFinal) + 0.5 * inertieDe * dot(vitesseAngFinal, vitesseAngFinal) + masseDe * abs(accelerationG(3)) + hauteur(3)
               energieMin = sqrt(2) * masseDe * abs(accelerationG(3)) * longueurCote

              if(conditionArret < energieMin  || (vitesseLinFinal(1) < 0 && vitesseLinFinal(2) < 0))
                sommets = calculPositionAngulaire(q, indice)
                test = calculFaceDuDessus(sommets)
                break;
              endif

            endif
          endif

      elseif(conditionCollision <= 0.001 && vitesseZ < 0)
            t(end) = [];
            x(end) = [];
            y(end) = [];
            z(end) = [];
            vX(end) = [];
            vY(end) = [];
            vZ(end) = [];
            indice = indice - 1;
            q = [x(indice) y(indice) z(indice) vX(indice) vY(indice) vZ(indice) q(7) q(8) q(9) q(10) q(11) q(12) q(13) q(14) q(15) q(16) q(17) q(18)];
            deltaTemps = deltaTemps / 2;
     endif
    indice = indice + 1;


   endwhile




