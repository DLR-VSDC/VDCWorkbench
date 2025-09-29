within VDCWorkbenchModels.VehicleComponents.Chassis.Components;
model SimpleAxisFront "Simple steerable axle"
  extends BaseClasses.InterfacesAxle;

  parameter Modelica.Units.SI.Length trackWidth=1.45
    "Track width of the vehicle";
  parameter Modelica.Units.SI.Length R0 "Radius of the wheel";
  parameter Modelica.Units.SI.Length s[2]={1,0}
    "Driving direction of the wheel at angle phi = 0";
  parameter Modelica.Units.SI.Force N "Normal force";
  parameter Modelica.Units.SI.Velocity vAdhesion "Adhesion velocity";
  parameter Modelica.Units.SI.Velocity vSlide "Sliding velocity";
  parameter Real mu_A "Friction coefficient at adhesion";
  parameter Real mu_S "Friction coefficient at sliding";
  parameter Modelica.Units.SI.Inertia J_wheel "Inertia of wheel";
  parameter Modelica.Units.SI.Inertia J_steer
    "Inertia of steering rack pinion mechanism";
  parameter Real steeringRatio = 16 "Steering gear ratio (phi_steeringWheel/phi_rack)";
  parameter Modelica.Units.SI.Velocity v_long
    "Velocity in longitudinal direction";

  PlanarMechanics.Joints.Revolute bearingLeft(
    useFlange=true) annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-34,-40})));
  PlanarMechanics.Joints.Revolute bearingRight(
    useFlange=true) annotation (
      Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={34,-40})));
  PlanarMechanics.Parts.FixedTranslation fixWheelLeft(
    r={0,-trackWidth/2}) annotation (Placement(transformation(extent={{-30,-70},{-10,-50}})));
  PlanarMechanics.Parts.FixedTranslation fixWheelRight(
    r={0,trackWidth/2}) annotation (Placement(transformation(extent={{30,-70},{10,-50}})));
  PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint wheelLeft(
    radius=R0,
    stateSelect=StateSelect.prefer,
    r=s,
    N=N,
    vAdhesion=vAdhesion,
    vSlide=vSlide,
    mu_A=mu_A,
    mu_S=mu_S,
    w_roll(start=v_long/R0, fixed=true),
    v_long(start=v_long),
    useHeatPort=useHeatPort)
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=180,
        origin={-50,0})));
  PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint wheelRight(
    radius=R0,
    r=s,
    N=N,
    vAdhesion=vAdhesion,
    vSlide=vSlide,
    mu_A=mu_A,
    mu_S=mu_S,
    w_roll(start=v_long/R0),
    v_long(start=v_long),
    useHeatPort=useHeatPort)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,0})));
  Modelica.Mechanics.Rotational.Components.Inertia wheelInertiaL(J=J_wheel)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia wheelInertiaR(J=J_wheel)
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia inertiaSteering(
    J=J_steer,
    stateSelect=StateSelect.prefer)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={10,10})));
  Modelica.Mechanics.Rotational.Components.IdealGear steeringGear(
    ratio=steeringRatio) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={10,50})));
  Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation (Placement(transformation(extent={{30,70},{50,90}})));
  PlanarMechanics.VehicleComponents.DifferentialGear differentialGear
    annotation (Placement(transformation(extent={{-30,40},{-10,20}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeSteeringWheel
    "Steernig wheel flange" annotation (Placement(transformation(extent={{30,90},{50,110}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDifferential
    "Input flange of differential" annotation (Placement(transformation(extent={{-50,90},{-30,110}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor tyreLossFL
    annotation (Placement(transformation(extent={{-56,-30},{-76,-50}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor tyreLossFR
    annotation (Placement(transformation(extent={{56,-30},{76,-50}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
equation
  connect(fixWheelLeft.frame_b, frameChassis) annotation (Line(
      points={{-10,-60},{0,-60},{0,-100}},
      color={95,95,95},
      thickness=0.5));
  connect(fixWheelLeft.frame_a, bearingLeft.frame_a) annotation (Line(
      points={{-30,-60},{-34,-60},{-34,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(wheelLeft.frame_a, bearingLeft.frame_b) annotation (Line(
      points={{-46,-4.44089e-16},{-34,-4.44089e-16},{-34,-30}},
      color={95,95,95},
      thickness=0.5));
  connect(bearingRight.frame_b,wheelRight. frame_a) annotation (Line(
      points={{34,-30},{34,0},{46,0}},
      color={95,95,95},
      thickness=0.5));
  connect(bearingLeft.axis, bearingRight.axis)
    annotation (Line(points={{-24,-40},{24,-40}}, color={0,0,0}));
  connect(wheelInertiaL.flange_b, wheelLeft.flange_a)
    annotation (Line(points={{-70,0},{-62,0},{-62,1.33227e-15},{-60,1.33227e-15}}, color={0,0,0}));
  connect(wheelRight.flange_a, wheelInertiaR.flange_a)
    annotation (Line(points={{60,0},{70,0}}, color={0,0,0}));
  connect(flangeWheelRight, wheelInertiaR.flange_b)
    annotation (Line(points={{100,0},{90,0}}, color={0,0,0}));
  connect(bearingLeft.axis, inertiaSteering.flange_b)
    annotation (Line(points={{-24,-40},{10,-40},{10,-3.55271e-15}}, color={0,0,0}));
  connect(flangeSteeringWheel, steeringGear.flange_a)
    annotation (Line(points={{40,100},{10,100},{10,60}}, color={0,0,0}));
  connect(steeringGear.flange_b, inertiaSteering.flange_a)
    annotation (Line(points={{10,40},{10,20}}, color={0,0,0}));
  connect(fixWheelRight.frame_b, frameChassis) annotation (Line(
      points={{10,-60},{0,-60},{0,-100}},
      color={95,95,95},
      thickness=0.5));
  connect(fixWheelRight.frame_a, bearingRight.frame_a) annotation (Line(
      points={{30,-60},{34,-60},{34,-50}},
      color={95,95,95},
      thickness=0.5));
  connect(wheelLeft.heatPort, tyreLossFL.port_a) annotation (Line(points={{-40,-10},{-40,-20},{-50,-20},{-50,-40},{-56,-40}}, color={191,0,0}));
  connect(wheelRight.heatPort, tyreLossFR.port_a)
    annotation (Line(points={{40,-10},{40,-20},{50,-20},{50,-40},{56,-40}}, color={191,0,0}));
  connect(wheelInertiaL.flange_a, flangeWheelLeft) annotation (Line(points={{-90,0},{-96,0},{-96,0},{-100,0}}, color={0,0,0}));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{60,60},{60,60.1},{99.9,60.1}},
      color={255,204,51},
      thickness=0.5));
  connect(flangeSteeringWheel, angleSensor.flange) annotation (Line(points={{40,100},{10,100},{10,80},{30,80}}, color={0,0,0}));
  connect(angleSensor.phi, chassisBus.steeringWheelAngle) annotation (Line(points={{51,80},{60,80},{60,60}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(differentialGear.flange_b, flangeDifferential) annotation (Line(points={{-20,40},{-20,100},{-40,100}}, color={0,0,0}));
  connect(differentialGear.flange_left, wheelLeft.flange_a) annotation (Line(
        points={{-30,30},{-64,30},{-64,0},{-62,0},{-62,1.33227e-15},{-60,1.33227e-15},{-60,7.21645e-16}}, color={0,0,0}));
  connect(differentialGear.flange_right, wheelInertiaR.flange_a)
    annotation (Line(points={{-10,30},{64,30},{64,0},{70,0}}, color={0,0,0}));
  connect(tyreLossFL.port_b, internalHeatPort) annotation (Line(points={{-76,-40},{-80,-40},{-80,-60},{-100,-60},{-100,-80}}, color={191,0,0}));
  connect(tyreLossFR.port_b, internalHeatPort) annotation (Line(points={{76,-40},{80,-40},{80,-70},{-50,-70},{-50,-60},{-100,-60},{-100,-80}}, color={191,0,0}));
  connect(tyreLossFL.Q_flow, chassisBus.tyreLossFL) annotation (Line(points={{-66,-29},{-66,70},{40,70},{40,60},{60,60}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(tyreLossFR.Q_flow, chassisBus.tyreLossFR) annotation (Line(points={{66,-29},{66,50},{60,50},{60,60}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-50,-30},{-30,-30}}, color={135,135,135}),
        Line(points={{30,20},{50,20}}, color={135,135,135}),
        Line(points={{30,-20},{50,-20}}, color={135,135,135}),
        Line(points={{-40,-30},{-40,-44},{40,-44},{40,-20}}, color={135,135,135}),
        Line(points={{0,-44},{0,-60}}, color={135,135,135}),
        Line(
          points={{-20,18},{-20,100},{-40,100}},
          color={0,0,0},
          thickness=1),
        Rectangle(
          extent={{64,12},{-6,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Rectangle(
          extent={{-100,60},{-60,-60}},
          lineColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Line(
          points={{-100,20},{-60,20}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,-20},{-60,-20}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,40},{-60,40}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,60},{-60,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,50},{-60,50}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,56},{-60,56}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,-50},{-60,-50}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,-60},{-60,-60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,-56},{-60,-56}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{-100,-40},{-60,-40}},
          color={95,95,95},
          smooth=Smooth.None),
        Rectangle(
          extent={{60,60},{100,-60}},
          lineColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Line(
          points={{60,20},{100,20}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-20},{100,-20}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,40},{100,40}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,60},{100,60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,50},{100,50}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,56},{100,56}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-50},{100,-50}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-60},{100,-60}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-56},{100,-56}},
          color={95,95,95},
          smooth=Smooth.None),
        Line(
          points={{60,-40},{100,-40}},
          color={95,95,95},
          smooth=Smooth.None),
        Ellipse(
          extent={{-78,8},{-62,-8}},
          lineColor={0,0,0},
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{62,8},{78,-8}},
          lineColor={0,0,0},
          fillColor={0,128,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-70,0},{-50,30},{50,30},{70,0}},
          color={0,0,0},
          thickness=1),
        Line(
          points={{40,100},{40,30}},
          color={0,0,0},
          thickness=1),
        Rectangle(
          extent={{0,22},{-40,-18}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Rectangle(
          extent={{-12,16},{-40,-12}},
          fillPattern=FillPattern.Solid,
          fillColor={255,255,255},
          pattern=LinePattern.None),
        Rectangle(
          extent={{-20,12},{-60,-8}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Line(
          points={{-40,72},{-34,72},{-32,66}},
          color={28,108,200},
          thickness=1),
        Ellipse(
          extent={{-38,78},{-2,42}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=220,
          closure=EllipseClosure.None),
        Text(
          extent={{-150,-60},{150,-100}},
          textColor={0,0,255},
          textString="%name")}),
    Documentation(info="<html>
<p>
An axle connecting rigidly the left and right wheels which can, additionally, be parallel steered.
A&nbsp;driving torque can be applied by the connector <code>flangeMotor</code> which
is divided to the wheels by a&nbsp;differential. Optionally, the left and right torque
can be applied directly on the wheel flanges.
</p>
</html>"));
end SimpleAxisFront;
