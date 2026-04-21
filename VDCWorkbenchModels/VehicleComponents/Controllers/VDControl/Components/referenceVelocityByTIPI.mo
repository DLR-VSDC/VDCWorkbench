within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
function referenceVelocityByTIPI "Calculate reference velocity using time-independet path interpolation"
  extends Modelica.Icons.Function;

  input Real Path[:,6]; // [sPath, posXPath, posYPath, posPsiPath, vLongPath, kPath]
  input Real arc_length;
  input Real vehStates[6];

  output Real arc_length_deriv;
  //output Real posPath[6];  // [sPath, posXPath, posYPath, posPsiPath, vLongPath, kPath]

  parameter Real lf = 0.1805;
  parameter Real ePathTanGain = 80;

protected
  Real posX;      // in inertial frame
  Real posY;      // in inertial frame
  Real posPsi;    // in inertial frame
  Real velX;      // in inertial frame
  Real velY;      // in inertial frame

  Integer nPath;
  Integer idxPath;
  Real denom;

  Real frac;
  Real posXPath;
  Real posYPath;
  Real posPsiPath;
  //Real vLongPath;
  Real kappaPath;

  Real posXFront;
  Real posYFront;

  Real deltaPosX;
  Real deltaPosY;

  Real ePathTan;
  Real ePathOrth;

algorithm
  // define variables
  posX   := vehStates[1];
  posY   := vehStates[2];
  posPsi := vehStates[3];
  velX   := cos(posPsi) * vehStates[4] - sin(posPsi) * vehStates[5];
  velY   := sin(posPsi) * vehStates[4] + cos(posPsi) * vehStates[5];

  // find position on path
  nPath := size(Path, 1);
  idxPath := 1;
  for i in 1:nPath loop
    if Path[i, 1] <= arc_length then
      idxPath := i;
    else
      break;
    end if;
  end for;

  frac := (arc_length - Path[idxPath, 1]) / (Path[idxPath + 1, 1] - Path[idxPath, 1]);

  posXPath   := Path[idxPath, 2] + frac * (Path[idxPath + 1, 2] - Path[idxPath, 2]);
  posYPath   := Path[idxPath, 3] + frac * (Path[idxPath + 1, 3] - Path[idxPath, 3]);
  posPsiPath := Path[idxPath, 4] + frac * (Path[idxPath + 1, 4] - Path[idxPath, 4]);
  kappaPath  := Path[idxPath, 6] + frac * (Path[idxPath + 1, 6] - Path[idxPath, 6]);

  // calc vehicle position on center of front axle
  posXFront := posX + lf * cos(posPsi);
  posYFront := posY + lf * sin(posPsi);

  // calc tangential and orthogonal error
  deltaPosX := posXFront - posXPath;
  deltaPosY := posYFront - posYPath;

  ePathTan  := deltaPosX*cos(posPsiPath) + deltaPosY*sin(posPsiPath);
  ePathOrth := -deltaPosX*sin(posPsiPath) + deltaPosY*cos(posPsiPath);

  // Calculate reference velocity
  arc_length_deriv :=(cos(posPsiPath)*velX + sin(posPsiPath)*velY)/(1 - ePathOrth*
    kappaPath) + ePathTanGain * ePathTan;

end referenceVelocityByTIPI;
