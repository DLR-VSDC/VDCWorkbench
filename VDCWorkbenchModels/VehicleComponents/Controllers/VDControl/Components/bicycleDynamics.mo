within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
function bicycleDynamics
  extends Modelica.Icons.Function;

  input Real states[6] "Vehicle states (x, y, psi, vx, vy, r)";
  input Real u[2] "Vehicle input signals (delta_f, torque)";

  input Modelica.Units.SI.Mass m = 7.151 "Total mass of vehicle";
  input Modelica.Units.SI.Inertia Jz = 1/12 * 7.151 * (0.265^2 + 0.361^2) "Yaw inertia of vehicle";
  input Modelica.Units.SI.Length lf = 0.1805 "Distance of center of mass to front axle (always positive here)";
  input Modelica.Units.SI.Length lr = 0.1805 "Distance of center of mass to rear axle (always positive here)";
  input Real C_Tire(unit="N/rad") = 90 "Tire cornering stiffness";
  input Modelica.Units.SI.Radius R0 = 0.0525 "Undeflected radius of wheel";
  input Real gearRatio = (48 * 43) / (16 * 11) "Total ratio between motor and wheel";

  output Real der_states[6] "First derivative of states (dx, dy, dpsi, dvx, dvy, dr)";

protected
  Real x, y, psi, vx, vy, r;
  Real delta_f, Fx;
  Real alpha_f, alpha_r, Fyf, Fyr;
  Real dx, dy, dpsi, dvx, dvy, dr;

algorithm
  x     := states[1];
  y     := states[2];
  psi   := states[3];
  vx    := states[4];
  vy    := states[5];
  r     := states[6];

  delta_f := u[1];
  Fx     := u[2] * gearRatio * R0;

  alpha_f := delta_f - atan2((vy + lf * r), max(vx, 0.1));
  alpha_r := -atan2((vy - lr * r), max(vx, 0.1));

  Fyf := C_Tire * alpha_f;
  Fyr := C_Tire * alpha_r;

  dx   := vx * cos(psi) - vy * sin(psi);
  dy   := vx * sin(psi) + vy * cos(psi);
  dpsi := r;

  dvx  := (Fx - Fyf * sin(delta_f)) / m + vy * r;
  dvy  := (Fyf * cos(delta_f) + Fyr) / m - vx * r;
  dr   := (Fyf * cos(delta_f) * lf - Fyr * lr) / Jz;

  der_states := {dx, dy, dpsi, dvx, dvy, dr};
end bicycleDynamics;
