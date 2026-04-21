within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
function bicycleDynamics
  extends Modelica.Icons.Function;

  input Real states[6];     //[x, y, psi, vx, vy, r]
  input Real u[2];          //[delta_f, torque]

  parameter Real m = 7.151;
  parameter Real Jz = 1/12 * 7.151 * (0.265^2 + 0.361^2);
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 90;
  parameter Real R0 = 0.0525;
  parameter Real gearRatio = (48 * 43) / (16 * 11);

  output Real der_states[6]; //[dx, dy, dpsi, dvx, dvy, dr]

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
