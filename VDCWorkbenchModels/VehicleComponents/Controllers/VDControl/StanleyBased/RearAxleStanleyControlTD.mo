within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model RearAxleStanleyControlTD "Time-discrete rear axle Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  parameter Real K = 1 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;

  parameter Real deltaMax = 0.3 "Steering saturation [rad]";

  parameter Real K_vctrl = 0.5 "P gain of velocity controller";
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = 0.3;

  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

  parameter Real m = 7.151 "Vehicle mass [kg]";
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 150 "Cornering stiffness";

protected
  parameter Real wheelbase = lf + lr;

public
  Real e_lat;
  Real x_front,y_front,xf_ref, yf_ref, psi_f_ref, delta_kappa_ref;
  Real theta_r_star;
  Real delta_ff;
  Real delta_yaw;
  Real delta_raw;
  Real theta_ss_r;
  Real theta_ss_f;
  Real yawrate_ref;
  Real delta_steer;

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
    x_front :=xveh + lf*cos(psiveh);
    y_front :=yveh + lf*sin(psiveh);

    yawrate_ref :=vveh_long*kappa_path;

    theta_ss_r :=(m/(C_Tire*(1 + lr/lf)))*vveh_long*yawrate_ref;
    theta_ss_f :=(m/(C_Tire*(1 + lf/lr)))*vveh_long*yawrate_ref;

    xf_ref := x_path + wheelbase*cos(psi_path + theta_ss_r);
    yf_ref := y_path + wheelbase*sin(psi_path + theta_ss_r);

    delta_kappa_ref := Modelica.Math.atan((wheelbase*(yawrate_ref/max(1e-6, vveh_long)) - Modelica.Math.sin(theta_ss_r))/Modelica.Math.cos(theta_ss_r));
    psi_f_ref := psi_path + theta_ss_r + delta_kappa_ref;

    e_lat := (yf_ref - y_front)*Modelica.Math.cos(psi_f_ref) - (xf_ref - x_front) * Modelica.Math.sin(psi_f_ref);

    theta_r_star := Modelica.Math.atan2(Modelica.Math.sin(psi_path + theta_ss_r - psiveh), Modelica.Math.cos(psi_path + theta_ss_r - psiveh));

    // feed forward control
    delta_ff := Modelica.Math.atan((wheelbase*kappa_ff - Modelica.Math.sin(theta_ss_r))/Modelica.Math.cos(theta_ss_r));

    // yaw rate damping
    delta_yaw := k_d_yaw*(yawrate_ref - yaw_rate) + theta_ss_f;

    // steer response damping
    delta_steer := k_d_steer * (delta_km1 - delta_km2);

    // Rear Axle Stanley Control Law
    delta_raw := delta_ff + theta_r_star + Modelica.Math.atan(K*e_lat/(vveh_long + v_eps)) + delta_yaw + delta_steer;
    delta := min(max(delta_raw, -deltaMax), deltaMax);

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
