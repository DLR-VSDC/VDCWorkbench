within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
function prediction
  extends Modelica.Icons.Function;

  input Real input_vehStates[6];
  input Real input_tipiState;
  input Real input_inputs[2];   // {delta, torque}
  input Real dt;
  input Integer N;
  input Real Path[:,6];         // [sPath, posXPath, posYPath, posPsiPath, vLongPath, kPath]
  input Real input_delta_k0;
  input Real input_delta_meas;
  output Real deltaPred[N];

protected
  Real vehStates[6];
  parameter Real k = 10 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;
  parameter Real deltaMax = 0.3 "Steering saturation [rad]";
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = 0.3;
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real Kspeedctrl = 0.5 "P gain of velocity controller";
  parameter Real C_Tire = 90 "Tire stiffnes for slip angle compensation";
  parameter Real m = 7.151 "Vehicle mass [kg]";

  Real tipiState;
  Real inputs[2];

  Real vehStates_k1[6], vehStates_k2[6], vehStates_k3[6], vehStates_k4[6];
  Real tipiState_k1, tipiState_k2, tipiState_k3, tipiState_k4;

  Integer nPath;
  Integer idxPath;

  Real frac;
  Real posXPath;
  Real posYPath;
  Real posPsiPath;
  Real vLongPath;
  Real kappaPath;

  Real posXFront;
  Real posYFront;
  Real e_lat;
  Real yaw_rate;
  Real yawRate_path;
  Real delta_yaw;
  Real delta_steer;
  Real delta_meas;
  Real delta_meas_old;
  Real dpsi;
  Real psi_ss;
  Real e_psi;
  Real delta;
  Real torque;

algorithm
  vehStates := input_vehStates;
  tipiState := input_tipiState;
  inputs := input_inputs;
  delta_meas := input_delta_k0;
  delta_meas_old := input_delta_meas;

  for i in 1:N loop
    // Integrate vehicle and TIPI with RK4
    vehStates_k1 := dt*bicycleDynamics(vehStates, inputs);
    tipiState_k1 := dt*referenceVelocityByTIPI(Path, tipiState, vehStates);

    vehStates_k2 := dt*bicycleDynamics(vehStates + 0.5*vehStates_k1, inputs);
    tipiState_k2 := dt*referenceVelocityByTIPI(Path, tipiState + 0.5*tipiState_k1, vehStates + 0.5*vehStates_k1);

    vehStates_k3 := dt*bicycleDynamics(vehStates + 0.5*vehStates_k2, inputs);
    tipiState_k3 := dt*referenceVelocityByTIPI(Path, tipiState + 0.5*tipiState_k2, vehStates + 0.5*vehStates_k2);

    vehStates_k4 := dt*bicycleDynamics(vehStates + vehStates_k3, inputs);
    tipiState_k4 := dt*referenceVelocityByTIPI(Path, tipiState + tipiState_k3, vehStates + vehStates_k3);

    vehStates := vehStates + (vehStates_k1 + 2 * vehStates_k2 + 2 * vehStates_k3 + vehStates_k4) / 6;
    tipiState := tipiState + (tipiState_k1 + 2 * tipiState_k2 + 2 * tipiState_k3 + tipiState_k4) / 6;

    // find position on path
    nPath := size(Path, 1);
    idxPath := 1;
    for i in 1:nPath loop
      if Path[i, 1] <= tipiState then
        idxPath := i;
      else
        break;
      end if;
    end for;

    frac := (tipiState - Path[idxPath, 1]) / (Path[idxPath + 1, 1] - Path[idxPath, 1]);

    posXPath   := Path[idxPath, 2] + frac * (Path[idxPath + 1, 2] - Path[idxPath, 2]);
    posYPath   := Path[idxPath, 3] + frac * (Path[idxPath + 1, 3] - Path[idxPath, 3]);
    posPsiPath := Path[idxPath, 4] + frac * (Path[idxPath + 1, 4] - Path[idxPath, 4]);
    vLongPath  := Path[idxPath, 5] + frac * (Path[idxPath + 1, 5] - Path[idxPath, 5]);
    kappaPath  := Path[idxPath, 6] + frac * (Path[idxPath + 1, 6] - Path[idxPath, 6]);

    // Stanley control law
    // calc vehicle position on center of front axle
    posXFront := vehStates[1] + lf * cos(vehStates[3]);
    posYFront := vehStates[2] + lf * sin(vehStates[3]);

    // calc lateral errors
    e_lat :=-Modelica.Math.sin(posPsiPath)*(posXPath - posXFront) + Modelica.Math.cos(
       posPsiPath)*(posYPath - posYFront);

    // Slip angle compensation
    yawRate_path := vehStates[4] * kappaPath;
    psi_ss := (m / (C_Tire * (1 + lf / lr))) * vehStates[4] * yawRate_path;
    dpsi := posPsiPath - vehStates[3] - psi_ss;
    e_psi := Modelica.Math.atan2(Modelica.Math.sin(dpsi), Modelica.Math.cos(dpsi));

    // yaw rate damping
    yaw_rate := vehStates[6];
    delta_yaw := k_d_yaw * (yawRate_path - yaw_rate);

    // steer response damping
    delta_steer := k_d_steer * (delta_meas - delta_meas_old);
    delta_meas_old := delta_meas;

    // Stanley Controll Law
    delta :=e_psi + Modelica.Math.atan(k*e_lat/(vehStates[4] + v_eps)) + delta_yaw + delta_steer;
    delta :=min(max(delta, -deltaMax), deltaMax);

    // Torque P-controller
    torque :=min(vctrl_TorqueMax, max(-vctrl_TorqueMax, Kspeedctrl*(vLongPath - vehStates[4])));

    // set variables for next loop
    deltaPred[i] :=delta;
    delta_meas_old := delta_meas;
    delta_meas := delta;

  end for;
end prediction;
