% Cette fonction permet de trouver les paramètres du dé juste avant la collision

function calculParamDeAvCollision = calculParamDeAvCollision(q0, t0)
  a = [0;0;-9.81];
  aa = [0; 0; 0];
  longueurCote = 0.04;    % en m

  test = q0(6);

  calculParamDeAvCollision = [q0(4), q0(5), q0(6), a(1), a(2), a(3), aa(1), aa(2), aa(3),q0(8)*q0(16) - q0(9)*q0(13), q0(8)*q0(17) - q0(9)*q0(14), q0(8)*q0(18) - q0(9)*q0(15),q0(9)*q0(10) - q0(7)*q0(16),  q0(9)*q0(11)- q0(7)*q0(17), q0(9)*q0(12)- q0(7)*q0(18), q0(7)*q0(13) - q0(8)*q0(10),  q0(7)*q0(14)- q0(8)*q0(11), q0(7)*q0(15)- q0(8)*q0(12)
  ];

