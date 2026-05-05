within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model RearAxleStanleyControlTD "Time-discrete rear axle Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  import Modelica.Math.{cos,sin,atan,atan2};

  parameter Real k = 1 "Stanley gain";
  parameter Modelica.Units.SI.Velocity v_eps = 0.1  "Small velocity to avoid division by zero";
  parameter Real k_d_yaw = 0.14 "Factor for yaw rate related damping";
  parameter Real k_d_steer = 0.0 "Factor penalizing rate of steering angle change";

  parameter Modelica.Units.SI.Angle deltaMax = 0.3 "Steering saturation";

  parameter Real K_vctrl = 0.5 "P gain of velocity controller" annotation (Dialog(group="Torque controller"));
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = 0.3 "Torque limit" annotation (Dialog(group="Torque controller"));

  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

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
  Real yawrate_ref;
  Real delta_ff;
  Real delta_raw;
  Real delta_yaw;
  Real delta_steer;
  Real theta_ss_r;
  Real theta_ss_f;

  discrete Real delta_km1(start=0);
  discrete Real delta_km2(start=0);

protected
  Modelica.Blocks.Interfaces.RealOutput kappa_ff
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-80})));

algorithm
  when sample(0, Ts) then

    // set coordinates to center of front axle
    x_front := xveh + lf*cos(psiveh);
    y_front := yveh + lf*sin(psiveh);

    yawrate_ref := vveh_long * kappa_path;

    theta_ss_r := m / (C_Tire*(1 + lr/lf)) * vveh_long * yawrate_ref;
    theta_ss_f := m / (C_Tire*(1 + lf/lr)) * vveh_long * yawrate_ref;

    xf_ref := x_path + wheelbase * cos(psi_path + theta_ss_r);
    yf_ref := y_path + wheelbase * sin(psi_path + theta_ss_r);

    delta_kappa_ref := atan( (wheelbase*(yawrate_ref/max(1e-6, vveh_long)) - sin(theta_ss_r)) / cos(theta_ss_r));
    psi_f_ref := psi_path + theta_ss_r + delta_kappa_ref;

    e_lat := -(xf_ref - x_front)*sin(psi_f_ref) + (yf_ref - y_front)*cos(psi_f_ref);

    theta_r_star := atan2( sin(psi_path + theta_ss_r - psiveh), cos(psi_path + theta_ss_r - psiveh) );

    // feed forward control
    delta_ff := atan( (wheelbase*kappa_ff - sin(theta_ss_r)) / cos(theta_ss_r) );

    // yaw rate damping
    delta_yaw := k_d_yaw * (yawrate_ref - yaw_rate) + theta_ss_f;

    // steer response damping
    delta_steer := k_d_steer * (delta_km1 - delta_km2);

    // Rear axle Stanley control law
    delta_raw := delta_ff + theta_r_star + atan(k * e_lat/(vveh_long + v_eps)) + delta_yaw + delta_steer;
    delta := min(deltaMax, max(-deltaMax, delta_raw));

    torque := min(vctrl_TorqueMax, max(-vctrl_TorqueMax, K_vctrl*(v_path - vveh_long)));

    delta_km2 := delta_km1;
    delta_km1 := delta;

  end when;
equation
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
Stanley"),
        Line(
          points={{-80,-80},{-80,-40},{-30,-40},{-30,-20},{20,-20},{20,-60},{70,-60},{70,-10}},
          color={255,255,255},
          pattern=LinePattern.Dot),
        Ellipse(
          extent={{-86,-34},{-74,-46}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,-14},{-24,-26}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-54},{26,-66}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end RearAxleStanleyControlTD;
