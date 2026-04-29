within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
function referenceVelocityByTIPI "Calculate reference velocity using time-independet path interpolation"
  extends Modelica.Icons.Function;

  input Real Path[:,6] "Path data table (sPath, posXPath, posYPath, posPsiPath, vLongPath, kPath)";
  input Real arc_length "Current arc length";
  input Real states[6] "Vehicle states (x, y, psi, vx, vy, r)";

  input Real lf = 0.1805 "Distance from vehicle's CoG to front axle";
  input Real ePathTanGain = 80;

  output Real arc_length_deriv;
  //output Real posPath[6];  // [sPath, posXPath, posYPath, posPsiPath, vLongPath, kPath]

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
  posX   := states[1];
  posY   := states[2];
  posPsi := states[3];
  velX   := cos(posPsi) * states[4] - sin(posPsi) * states[5];
  velY   := sin(posPsi) * states[4] + cos(posPsi) * states[5];

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
