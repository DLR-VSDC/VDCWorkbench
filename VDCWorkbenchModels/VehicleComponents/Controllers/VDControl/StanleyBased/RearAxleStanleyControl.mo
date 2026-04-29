within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model RearAxleStanleyControl "Rear axle Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  import Modelica.Math.{cos,sin,atan,atan2};

  parameter Real k = 1 "Stanley gain";
  parameter Modelica.Units.SI.Velocity v_eps = 0.1  "Small velocity to avoid division by zero";
  parameter Real k_d_yaw = 0.14 "Factor for yaw rate related damping";
  parameter Real k_d_steer = 0.0 "Factor penalizing rate of steering angle change";

  parameter Modelica.Units.SI.Angle deltaMax = 0.3 "Steering saturation";

  parameter Real K_vctrl = 0.5 "Gain of torque control" annotation (Dialog(group="Drive torque controller"));
  parameter Modelica.Units.SI.Torque tauDriveMax = 0.3 "Torque limit" annotation (Dialog(group="Drive torque controller"));

  parameter Modelica.Units.SI.Mass m = 7.151 "Vehicle mass" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lf = 0.1805 "Distance of CoG to front axle" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lr = 0.1805 "Distance of CoG to rear axle" annotation(Dialog(group="Vehicle parameters"));
  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation" annotation(Dialog(group="Vehicle parameters"));

protected
  parameter Modelica.Units.SI.Length wheelbase = lf + lr;

public
  Real e_lat;
  Real x_front,y_front,xf_ref, yf_ref, psi_f_ref, delta_kappa_ref;
  Real theta_r_star;
  Real yawRate_path;
  Real delta_raw;
  Real delta_yaw;
  Real delta_ff;
  Real theta_ss_r;
  Real theta_ss_f;

protected
  Modelica.Blocks.Interfaces.RealOutput kappa_ff
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-80})));

equation

  // set coordinates to center of front axle
  x_front = xveh + lf*cos(psiveh);
  y_front = yveh + lf*sin(psiveh);

  yawRate_path  = vveh_long * kappa_path;

  theta_ss_r = m / (C_Tire * (1 + lr/lf)) * vveh_long * yawRate_path;
  theta_ss_f = m / (C_Tire * (1 + lf/lr)) * vveh_long * yawRate_path;

  xf_ref = x_path + wheelbase * cos(psi_path + theta_ss_r);
  yf_ref = y_path + wheelbase * sin(psi_path + theta_ss_r);

  delta_kappa_ref = atan( (wheelbase*(yawRate_path / max(1e-6, vveh_long)) - sin(theta_ss_r)) / cos(theta_ss_r));
  psi_f_ref = psi_path + theta_ss_r + delta_kappa_ref;

  e_lat = -(xf_ref - x_front)*sin(psi_f_ref) + (yf_ref - y_front)*cos(psi_f_ref);

  theta_r_star = atan2(sin(psi_path + theta_ss_r - psiveh), cos(psi_path + theta_ss_r - psiveh));

  // feed forward control
  delta_ff = atan((wheelbase*kappa_ff - sin(theta_ss_r)) / cos(theta_ss_r));

  // yaw rate damping
  delta_yaw = k_d_yaw * (yawRate_path - yaw_rate) + theta_ss_f;

  // steer response damping (not defined for model without steering dynamics)
  //delta_steer = k_d_steer * (delta_km1 - delta_km2);

  // Rear axle Stanley control law
  delta_raw = delta_ff + theta_r_star + atan(k * e_lat/(vveh_long + v_eps)) + delta_yaw; // + delta_steer
  delta = min(deltaMax, max(-deltaMax, delta_raw));

  torque = min(tauDriveMax, max(-tauDriveMax, K_vctrl*(v_path - vveh_long)));

  connect(kappa_ff, motionDemandBus.kappa_ff) annotation (Line(points={{-30,-80},{10,-80},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));

  annotation (
    Icon(
      graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={217,67,180},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,60},{100,0}},
          textColor={255,255,255},
          textString="rear axle
Stanley")}),
    Documentation(
      info="<html>
<p>
Path following <em>Stanley</em> controller which uses the <em>rear axle</em> as the reference
point for path tracking. To compute the lateral and the heading errors defined at the front axle,
required for the
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased.StanleyControl\">Stanley control law</a>,
a&nbsp;dynamic single-track reference model is employed which projects the rear axle reference
state onto the path while accounting for vehicle geometry and rear axle slip angle.
The full derivation and implementation details are described in
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Seiffer2023</a>].
</p>
<p>
The control law, including the defined tracking errors, can be reformulated as the sum of the
reference model&apos;s steering angle &delta;<sub>ref</sub> and additional feedback terms.
Since &delta;<sub>ref</sub> depends solely on the reference path and the vehicle&apos;s kinematic
model, it acts as a&nbsp;feedforward component of the control law. This feedforward term can
be leveraged to compensate for system delays (if they exist) by shifting the reference model
along the path by a&nbsp;distance <var>s</var> = <var>v<sub>x</sub></var> <var>T</var><sub>ff</sub>,
where <var>v<sub>x</sub></var> denotes the vehicle&apos;s longitudinal velocity and
<var>T</var><sub>ff</sub> represents the system delay.
The resulting control output <code>delta</code> is bounded to [-&pi;, &pi;] and further saturated
with respect to the actuator limits <code>deltaMax</code>.
</p>
<p>
The output torque demand <code>torque</code> of propulsion is proportional to the deviation of
vehicle&apos;s longitudinal velocity by factor <code>K_vctrl</code> and limited by
<code>tauDriveMax</code>.
</p>
</html>"));
end RearAxleStanleyControl;
