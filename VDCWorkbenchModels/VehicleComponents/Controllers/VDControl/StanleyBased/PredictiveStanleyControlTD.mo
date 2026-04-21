within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model PredictiveStanleyControlTD "Time-discrete predictive Stanley lateral control law"
  parameter Real k = 5 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;

  parameter Real deltaMax = 0.3 "Steering saturation [rad]";

  parameter Real K_vctr = 0.5 "P gain of velocity controller";
  parameter Modelica.Units.SI.Torque vctr_TorqueMax = 0.3;

  parameter Real Ts = 0.05 "Controller sample time [s]";

  parameter Real m = 7.151 "Vehicle mass [kg]";
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation";

  parameter Boolean use_prediction = true;
  parameter Real weights[5] = {0.4, 0.2, 0.2, 0.1, 0.1};
  parameter Integer N = 4;
  parameter Real dt = Ts;

  // ===================== Path data =====================
  parameter Integer nPath = 4999 "Number of path points";
  parameter String FilePath = ModelicaServices.ExternalReferences.loadResource(
    "modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat")
    "File where path table information is stored in table 'path'";
  parameter String pathName = "path" "Table name in .mat file";
  parameter Real pathData[nPath, 6] = Modelica.Utilities.Streams.readRealMatrix(
    FilePath, pathName, nPath, 6);
  parameter Real maxArc = pathData[nPath, 1];

public
  Real x_front, y_front;
  Real v_lat;
  Real e_lat;
  Real yawRate_path;
  Real delta_k0;
  Real delta_raw;
  Real e_psi;
  Real delta_yaw;
  Real delta_steer;
  Real dpsi;
  Real psi_ss;

  discrete Real delta_km1(start=0);
  discrete Real delta_km2(start=0);

  Real inputs[2];
  Real vehStates[6];

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
  Modelica.Blocks.Interfaces.RealOutput vveh "Absolute vehicle speed" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-40})));
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
  Modelica.Blocks.Interfaces.RealOutput yawRate annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,40})));
  Modelica.Blocks.Interfaces.RealOutput v_dx annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,-60})));
  Modelica.Blocks.Interfaces.RealOutput arcLength annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,60})));
  Modelica.Blocks.Interfaces.RealOutput v_dy annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={40,-80})));

algorithm
  when sample(0, Ts) then
    // set coordinates to center of front axle
    x_front :=xveh + lf*cos(psiveh);
    y_front :=yveh + lf*sin(psiveh);

    v_lat := - v_dx * sin(psiveh) + v_dy * cos(psiveh);

    // calc errors
    e_lat :=-Modelica.Math.sin(psi_path)*(x_path - x_front) + Modelica.Math.cos(
       psi_path)*(y_path - y_front);

    // Slip angle compensation
    yawRate_path := vveh * kappa_path;
    psi_ss := (m / (C_Tire * (1 + lf / lr))) * vveh * yawRate_path;
    dpsi := psi_path - psiveh - psi_ss;
    e_psi := Modelica.Math.atan2(Modelica.Math.sin(dpsi), Modelica.Math.cos(dpsi));

    // yaw rate damping
    delta_yaw :=k_d_yaw*(yawRate_path - yawRate);

    // steer response damping
    delta_steer := k_d_steer * (delta_km1 - delta_km2);

    // Stanley Controll Law
    delta_k0 := e_psi + Modelica.Math.atan(k*e_lat/(vveh + v_eps)) + delta_yaw + delta_steer;
    delta_k0 := min(max(delta_k0, -deltaMax), deltaMax);

    torque :=min(vctr_TorqueMax, max(- vctr_TorqueMax, K_vctr*(v_path - vveh)));

    delta_km2 := delta_km1;
    delta_km1 := delta;

    // prediction part of model
    if use_prediction then
      vehStates :={xveh,yveh,psiveh,vveh,v_lat,yawRate};
      inputs :={delta_k0, torque};
      delta_raw :=weights[1]*delta_k0 + weights[2:(N + 1)]*Components.prediction(
        vehStates,
        arcLength,
        inputs,
        dt,
        N,
        pathData,
        delta_k0,
        delta_km1);
    else
      delta_raw := delta_k0;
    end if;

    delta :=min(max(delta_raw, -deltaMax), deltaMax);

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
  connect(vveh, chassisBus.longitudinalVelocity) annotation (Line(points={{40,-40},
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
  connect(kappa_path, motionDemandBus.kappa_path) annotation (Line(points={{-80,
          40},{-8,40},{-8,70}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(yawRate, chassisBus.yawRate) annotation (Line(points={{40,40},{84,40},
          {84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(arcLength, motionDemandBus.arc_length) annotation (Line(points={{40,60},
          {-8,60},{-8,70}},
                         color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(v_dx, chassisBus.velocity_dx) annotation (Line(points={{40,-60},{84,
          -60},{84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(v_dy, chassisBus.velocity_dy) annotation (Line(points={{40,-80},{84,
          -80},{84,-32}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  annotation (Icon(graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={244,125,35},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,80},{100,-80}},
          textColor={255,255,255},
          textString="TD
PredStanley
Controller")}));
end PredictiveStanleyControlTD;
