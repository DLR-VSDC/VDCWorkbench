within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model RearAxleStanleyControlTD "Time-discrete rear axle Stanley lateral control law"
  parameter Real K = 1 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;

  parameter Real deltaMax = 0.3 "Steering saturation [rad]";

  parameter Real K_vctrl = 0.5 "P gain of velocity controller";
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = 0.3;

  parameter Real Ts = 0.05 "Controller sample time [s]";

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

public
  Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={100,0})));
  Modelica.Blocks.Interfaces.RealOutput torque "Summarized propulsion torque" annotation (Placement(transformation(extent={{100,28},{124,52}})));
  Modelica.Blocks.Interfaces.RealOutput delta annotation (Placement(transformation(extent={{100,66},{126,92}})));
protected
  Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-18,60},{2,80}})));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{74,-42},{94,-22}})));
  Modelica.Blocks.Interfaces.RealOutput xveh "Measured vehicle position, x" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,20})));
  Modelica.Blocks.Interfaces.RealOutput yveh "Measured vehicle position, y" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,0})));
  Modelica.Blocks.Interfaces.RealOutput psiveh "Measured vehicle position, psi" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-20})));
  Modelica.Blocks.Interfaces.RealOutput vveh_long
    "Absolute longitudinal vehicle speed" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,-40})));
  Modelica.Blocks.Interfaces.RealOutput psi_path
    "Position psi of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,20})));
  Modelica.Blocks.Interfaces.RealOutput x_path
    "Position x of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,0})));
  Modelica.Blocks.Interfaces.RealOutput y_path
    "Position y of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,-20})));
  Modelica.Blocks.Interfaces.RealOutput v_path
    "Position y of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,-40})));
  Modelica.Blocks.Interfaces.RealOutput kappa_ff
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,-60})));
  Modelica.Blocks.Interfaces.RealOutput kappa_path annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,-80})));
  Modelica.Blocks.Interfaces.RealOutput yawRate annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,40})));

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
    delta_yaw := k_d_yaw*(yawrate_ref - yawRate) + theta_ss_f;

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
  connect(controlBus.motionDemandBus, motionDemandBus) annotation (Line(
      points={{100.1,-0.1},{102,-0.1},{102,2},{94,2},{94,70},{-8,70}},
      color={255,204,51},
      thickness=0.5));
  connect(xveh, chassisBus.position_x) annotation (Line(points={{40,20},{84,20},
          {84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{12,3},{12,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(yveh, chassisBus.position_y) annotation (Line(points={{40,0},{84,0},{84,
          -32}},     color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{12,3},{12,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(psiveh, chassisBus.yawAngle) annotation (Line(points={{40,-20},{84,-20},
          {84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{12,3},{12,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{84,-32},{94,-32},{94,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(vveh_long, chassisBus.longitudinalVelocity) annotation (Line(points={{
          40,-40},{84,-40},{84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(psi_path, motionDemandBus.psi_path) annotation (Line(points={{-80,20},
          {-8,20},{-8,70}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(x_path, motionDemandBus.x_path) annotation (Line(points={{-80,0},{-8,0},{-8,70}},            color={0,0,127}));
  connect(y_path, motionDemandBus.y_path) annotation (Line(points={{-80,-20},{-8,
          -20},{-8,70}},                                                                               color={0,0,127}));
  connect(v_path, motionDemandBus.v_path) annotation (Line(points={{-80,-40},{-8,
          -40},{-8,70}},                    color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(kappa_ff, motionDemandBus.kappa_ff) annotation (Line(points={{-80,-60},
          {-8,-60},{-8,70}},         color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(kappa_path, motionDemandBus.kappa_path) annotation (Line(points={{-80,-80},
          {-8,-80},{-8,70}},      color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
    horizontalAlignment=TextAlignment.Right));
  connect(yawRate, chassisBus.yawRate) annotation (Line(points={{40,40},{84,40},
          {84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-82,-78})),
                Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={38,-64})),
                Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={38,-84})),
              Icon(graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={217,67,180},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,80},{98,-80}},
          textColor={255,255,255},
          textString="TD
RA Stanley 
Controller")}));
end RearAxleStanleyControlTD;
