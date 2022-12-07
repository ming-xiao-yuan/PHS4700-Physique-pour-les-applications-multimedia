# Fonction qui permet de trouver le sommet qui est en contact avec le sol lors de la collision

function calculSommetCollision = calculSommetCollision(sommets)

  [M, I] = min(transpose(sommets));
  idZ = I(1,3);
  calculSommetCollision = transpose(transpose(sommets)(idZ,:));


end
