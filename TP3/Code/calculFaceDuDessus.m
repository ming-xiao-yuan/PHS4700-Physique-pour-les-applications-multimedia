# Fonction qui permet de trouver la face du dessus

function calculFaceDuDessus = calculFaceDuDessus(sommets)

  lastRow = sommets(3,:)
  value = [];
  for i =1:4
    [val, I] = max(lastRow);
    value =[value, I];
    lastRow(I) = -1;
  endfor

  sortedKey = sort(value)


  face1 = strcat([1 2 3 6])
  face2 = strcat([3 4 7 8])
  face3 = strcat([2 3 6 7])
  face4 = strcat([1 4 5 8])
  face5 = strcat([1 2 5 6])
  face6 = strcat([5 6 7 8])

  keyString = strcat(sortedKey)


  M = containers.Map(face1,1);
  M = containers.Map(face2,2);
  M = containers.Map(face3,3);
  M = containers.Map(face4,4);
  M = containers.Map(face5,5);
  M = containers.Map(face6,6);


  % face = M(keyString)
  calculFaceDuDessus = 0


