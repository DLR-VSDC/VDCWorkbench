within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model StanleyControl "Classic Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  import Modelica.Math.{cos,sin,atan,atan2};

  parameter Real k = 5 "Stanley gain";
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

public
  Real e_lat;
  Real e_psi;
  Real x_front, y_front;
  Real yawRate_path;
  Real delta_raw;
  Real delta_yaw;
  Real dpsi;
  Real psi_ss;

equation
  // set coordinates to center of front axle
  x_front = xveh + lf*cos(psiveh);
  y_front = yveh + lf*sin(psiveh);

  // calc errors
  e_lat = -(x_path - x_front)*sin(psi_path) + (y_path - y_front)*cos(psi_path);

  // Slip angle compensation
  yawRate_path = vveh_long * kappa_path;
  psi_ss = m / (C_Tire * (1 + lf/lr)) * vveh_long * yawRate_path;
  dpsi = psi_path - psiveh - psi_ss;
  e_psi = atan2(sin(dpsi), cos(dpsi));

  // yaw rate damping
  delta_yaw = k_d_yaw * (yawRate_path - yaw_rate);

  // steer response damping (not defined for model without steering dynamics)
  //delta_steer =  k_d_steer * (delta_km1 - delta_km2);

  // Stanley control law
  delta_raw = e_psi + atan(k * e_lat/(vveh_long + v_eps)) + delta_yaw; // + delta_steer;
  delta = min(deltaMax, max(-deltaMax, delta_raw));

  torque = min(tauDriveMax, max(-tauDriveMax, K_vctrl*(v_path - vveh_long)));

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
          textString="Stanley")}),
    Documentation(
      info="<html>
<p>
Path following <em>Stanley</em> controller, see
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Hoffmann2007</a>].
This type of controller minimize the lateral deviation <var>e<sub>y</sub></var>, defined
w.r.t. center of front vehicle&apos;s axle, and orientation error <var>e<sub>&psi;</sub></var>
similar to the
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.GeoPFC\">geometric path-following controller</a>.
The deviations are computed based on the current vehicle&apos;s position and the path geometry
at the corresponding path position.
</p>
<p>
Instead of the center of gravity, the implemented approach considers the control error at the
<em>center of the front axle</em>. The Stanley control law computes the desired steering angle as
</p>
<blockquote>
<img src=\"modelica://VDCWorkbenchModels/Resources/Images/equations/equation-5Bb5AaLh.png\" alt=\"delta_f = e_psi + arctan( (k * e_y)/(v_x + k_prot) ) + delta_add\"/>
</blockquote>
<p>
where <var>v<sub>x</sub></var> is the vehicle&apos;s longitudinal velocity, <var>k</var> is a&nbsp;user-defined parameter <code>k</code> and
<var>k<sub>prot</sub></var>&nbsp;&gt;&nbsp;0 a&nbsp;small constant (parameter <code>v_eps</code>)
to prevent numerical instability at low velocities.
The term &delta;<sub>add</sub>, which depends on parameter <code>k_d_yaw</code>, represents additional
damping term to suppress oscillatory behavior at higher vehicle&apos;s velocities.
The resulting control output <code>delta</code> is bounded to [-&pi;, &pi;] and further saturated
with respect to the actuator limits <code>deltaMax</code>.
The controller also contains a&nbsp;model-based correction to account for the difference
between the geometric wheel orientation and the actual vehicle heading due to lateral tire
forces.
</p>
<p>
The output torque demand <code>torque</code> of propulsion is proportional to the deviation of
vehicle&apos;s longitudinal velocity by factor <code>K_vctrl</code> and limited by
<code>tauDriveMax</code>.
</p>
</html>"));
end StanleyControl;
