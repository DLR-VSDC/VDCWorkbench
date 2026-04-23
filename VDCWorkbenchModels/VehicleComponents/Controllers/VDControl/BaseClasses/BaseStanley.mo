within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.BaseClasses;
partial model BaseStanley "Basic interfaces for Stanley-based controller"

  Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={100,0})));
  Modelica.Blocks.Interfaces.RealOutput torque "Summarized propulsion torque"
    annotation (Placement(transformation(extent={{100,30},{120,50}}), iconTransformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.RealOutput delta
    annotation (Placement(transformation(extent={{100,70},{120,90}}), iconTransformation(extent={{100,70},{120,90}})));
protected
  Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{60,20},{80,40}})));
  Modelica.Blocks.Interfaces.RealOutput xveh "Measured vehicle position, x"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={30,20})));
  Modelica.Blocks.Interfaces.RealOutput yveh "Measured vehicle position, y"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={30,0})));
  Modelica.Blocks.Interfaces.RealOutput psiveh "Measured vehicle position, psi"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={30,-20})));
  Modelica.Blocks.Interfaces.RealOutput vveh_long "Absolute longitudinal vehicle speed"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={30,-60})));
  Modelica.Blocks.Interfaces.RealOutput psi_path "Position psi of path at current arc length value"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-20})));
  Modelica.Blocks.Interfaces.RealOutput x_path "Position x of path at current arc length value"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,20})));
  Modelica.Blocks.Interfaces.RealOutput y_path "Position y of path at current arc length value"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,0})));
  Modelica.Blocks.Interfaces.RealOutput v_path "Velocity v at current arc length value"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-40})));
  Modelica.Blocks.Interfaces.RealOutput kappa_path "Curvature kappa of path at current arc length value"
    annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-60})));
  Modelica.Blocks.Interfaces.RealOutput yaw_rate "Yaw rate of chassis"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={30,-40})));

equation
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
        points={{10,30},{80,30},{80,-0.1},{100.1,-0.1}},
        color={255,204,51},
        thickness=0.5));
  connect(xveh, chassisBus.position_x) annotation (Line(
        points={{30,20},{70,20},{70,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(yveh, chassisBus.position_y) annotation (Line(points={{30,0},{70,0},{70,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(psiveh, chassisBus.yawAngle) annotation (Line(points={{30,-20},{70,-20},{70,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
        points={{70,30},{80,30},{80,-0.1},{100.1,-0.1}},
        color={255,204,51},
        thickness=0.5));
  connect(vveh_long, chassisBus.longitudinalVelocity) annotation (Line(points={{30,-60},{70,-60},{70,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(psi_path, motionDemandBus.psi_path) annotation (Line(points={{-30,-20},{10,-20},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(x_path, motionDemandBus.x_path) annotation (
      Line(
        points={{-30,20},{10,20},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(y_path, motionDemandBus.y_path) annotation (Line(points={{-30,0},{10,0},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(v_path, motionDemandBus.v_path) annotation (Line(points={{-30,-40},{10,-40},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(kappa_path, motionDemandBus.kappa_path) annotation (Line(points={{-30,-60},{10,-60},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(yaw_rate, chassisBus.yawRate) annotation (
      Line(
        points={{30,-40},{70,-40},{70,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{12,2},{12,5}},
        horizontalAlignment=TextAlignment.Left));
end BaseStanley;
