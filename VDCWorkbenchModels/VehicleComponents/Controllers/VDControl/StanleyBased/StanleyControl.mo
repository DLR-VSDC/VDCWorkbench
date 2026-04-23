within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model StanleyControl "Classic Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  parameter Real k = 5 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;

  parameter Real deltaMax = 0.3 "Steering saturation [rad]";

  parameter Real K_vctrl = 0.5 "P gain of velocity controller";
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = 0.3;

  parameter Real m = 7.151 "Vehicle mass [kg]";
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation";

public
  Real e_lat;
  Real delta_raw;
  Real x_front,y_front;
  Real e_psi;
  Real yawRate_path;
  Real delta_yaw;
  Real dpsi;
  Real psi_ss;

equation
  // set coordinates to center of front axle
  x_front = xveh + lf*cos(psiveh);
  y_front = yveh + lf*sin(psiveh);

  // calc errors
  e_lat = -Modelica.Math.sin(psi_path)*(x_path - x_front) + Modelica.Math.cos(
     psi_path)*(y_path - y_front);
  //e_psi = Modelica.Math.atan2(Modelica.Math.sin(psi_path - psiveh),
    //Modelica.Math.cos(psi_path - psiveh));

  // Slip angle compensation
  yawRate_path =   vveh_long * kappa_path;
  psi_ss =  (m / (C_Tire * (1 + lf / lr))) * vveh_long * yawRate_path;
  dpsi =  psi_path - psiveh - psi_ss;
  e_psi =  Modelica.Math.atan2(Modelica.Math.sin(dpsi), Modelica.Math.cos(dpsi));

  // yaw rate damping
  delta_yaw =  k_d_yaw * (yawRate_path - yaw_rate);

  // steer response damping (not defined for model without steering dynamics)
  //delta_steer =  k_d_steer * (delta_km1 - delta_km2);

  // Stanley Controll Law
  delta_raw = e_psi + Modelica.Math.atan(k*e_lat/(vveh_long + v_eps)) + delta_yaw; // + delta_steer;
  delta = min(max(delta_raw, -deltaMax), deltaMax);

  torque = max(- vctrl_TorqueMax, min(vctrl_TorqueMax, K_vctrl*(v_path - vveh_long)));

  annotation (
    Icon(
      graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={0,140,72},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,60},{100,30}},
          textColor={255,255,255},
          textString="Stanley")}));
end StanleyControl;
