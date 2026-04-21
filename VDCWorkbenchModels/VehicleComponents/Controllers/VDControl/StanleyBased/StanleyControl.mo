within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model StanleyControl "Classic Stanley lateral control law"
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
  Modelica.Blocks.Interfaces.RealOutput vveh_long "Absolute longitudinal vehicle speed" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-40})));
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

  Modelica.Blocks.Interfaces.RealOutput kappa_path
    "Position psi of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-80,40})));
  Modelica.Blocks.Interfaces.RealOutput yaw_rate "Absolute vehicle speed"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,40})));

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
  connect(vveh_long, chassisBus.longitudinalVelocity) annotation (Line(points={{40,-40},
          {84,-40},{84,-32}}, color={0,0,127}), Text(
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
  connect(x_path, motionDemandBus.x_path)
    annotation (Line(points={{-80,0},{-8,0},{-8,70}},            color={0,0,127}));
  connect(y_path, motionDemandBus.y_path) annotation (Line(points={{-80,-20},{-8,
          -20},{-8,70}},                                                                               color={0,0,127}));
  connect(v_path, motionDemandBus.v_path) annotation (Line(points={{-80,-40},{-8,
          -40},{-8,70}},                    color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
    horizontalAlignment=TextAlignment.Left));
  connect(kappa_path, motionDemandBus.kappa_path) annotation (Line(points={{-80,40},
          {-8,40},{-8,70}},     color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(yaw_rate, chassisBus.yawRate) annotation (Line(points={{40,40},{84,40},
          {84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  annotation (Icon(graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={0,140,72},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,80},{100,-80}},
          textColor={255,255,255},
          textString="Stanley 
Controller
")}));
end StanleyControl;
