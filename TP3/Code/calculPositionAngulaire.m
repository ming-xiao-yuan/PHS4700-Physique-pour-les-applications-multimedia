% Cette fonction permet de trouver les angles associés à la position angulaire du dé

function calculPositionAngulaire = calculPositionAngulaire(q0, indice)
        longueurCote = 0.04;

         mRotation = [ q0(10), q0(11), q0(12);
                    q0(13), q0(14), q0(15);
                    q0(16), q0(17), q0(18);
                   ];



        rC = [q0(1); q0(2); q0(3)];
        sommet1 = rC + mRotation * [longueurCote/2; longueurCote/2; -longueurCote/2];
        sommet2 = rC + mRotation * [-longueurCote/2; longueurCote/2; -longueurCote/2];
        sommet3 = rC + mRotation * [-longueurCote/2; -longueurCote/2; -longueurCote/2];
        sommet4 = rC + mRotation * [longueurCote/2; -longueurCote/2; -longueurCote/2];
        sommet5 = rC + mRotation * [longueurCote/2; longueurCote/2; longueurCote/2];
        sommet6 = rC + mRotation * [-longueurCote/2; longueurCote/2; longueurCote/2];
        sommet7 = rC + mRotation * [-longueurCote/2; -longueurCote/2; longueurCote/2];
        sommet8 = rC + mRotation * [longueurCote/2; -longueurCote/2; longueurCote/2];


        calculPositionAngulaire = [sommet1, sommet2, sommet3, sommet4, sommet5, sommet6, sommet7, sommet8];

end

