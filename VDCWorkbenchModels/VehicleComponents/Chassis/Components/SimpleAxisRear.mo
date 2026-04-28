within VDCWorkbenchModels.VehicleComponents.Chassis.Components;
model SimpleAxisRear "Simple non-steerable axle"
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
  parameter Modelica.Units.SI.Velocity v_long
    "Velocity in longitudinal direction";

  parameter Boolean animate=true "= true, if animation shall be enabled" annotation (Dialog(group="Animation"));
  parameter Modelica.Units.SI.Length width=1.2*R0 "Width of the wheel" annotation (Dialog(group="Animation"));

  PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint wheelLeft(
    stateSelect=StateSelect.prefer,
    r=s,
    N=N,
    vAdhesion=vAdhesion,
    vSlide=vSlide,
    mu_A=mu_A,
    mu_S=mu_S,
    radius=R0,
    w_roll(fixed=true, start=v_long/R0),
    v_long(start=v_long),
    useHeatPort=useHeatPort,
    animate=animate,
    diameter=1.04*width,
    width=width)
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
    useHeatPort=useHeatPort,
    animate=animate,
    diameter=wheelLeft.diameter,
    width=wheelLeft.width)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,0})));
  Modelica.Mechanics.Rotational.Components.Inertia wheelInertiaL(J=J_wheel)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));

  Modelica.Mechanics.Rotational.Components.Inertia wheelInertiaR(J=J_wheel)
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  PlanarMechanics.VehicleComponents.DifferentialGear differentialGear
    annotation (Placement(transformation(extent={{-10,40},{10,20}})));
  PlanarMechanics.Parts.FixedTranslation fixWheelLeft(
    r={0,-trackWidth/2},
    animate=animate) annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
  PlanarMechanics.Parts.FixedTranslation fixWheelRight(
    r={0,trackWidth/2},
    animate=animate) annotation (Placement(transformation(extent={{30,-10},{10,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDifferential
    "Input flange of differential" annotation (Placement(transformation(extent={{-10,90},{10,110}}), iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor tyreLossRR
    annotation (Placement(transformation(extent={{56,-30},{76,-50}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor tyreLossRL
    annotation (Placement(transformation(extent={{-56,-30},{-76,-50}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{40,50},{60,70}})));
equation
  connect(wheelInertiaL.flange_b, wheelLeft.flange_a)
    annotation (Line(points={{-70,0},{-60,0}}, color={0,0,0}));
  connect(wheelInertiaL.flange_a, flangeWheelLeft)
    annotation (Line(points={{-90,0},{-100,0}}, color={0,0,0}));
  connect(differentialGear.flange_left, wheelLeft.flange_a)
    annotation (Line(points={{-10,30},{-64,30},{-64,0},{-60,0}}, color={0,0,0}));
  connect(differentialGear.flange_b, flangeDifferential)
    annotation (Line(points={{0,40},{0,100}}, color={0,0,0}));
  connect(wheelLeft.frame_a, fixWheelLeft.frame_a) annotation (Line(
      points={{-46,-4.44089e-16},{-40,-4.44089e-16},{-40,0},{-30,0}},
      color={95,95,95},
      thickness=0.5));
  connect(fixWheelLeft.frame_b, frameChassis) annotation (Line(
      points={{-10,0},{0,0},{0,-100}},
      color={95,95,95},
      thickness=0.5));
  connect(wheelRight.flange_a, wheelInertiaR.flange_a)
    annotation (Line(points={{60,0},{70,0}}, color={0,0,0}));
  connect(differentialGear.flange_right, wheelInertiaR.flange_a)
    annotation (Line(points={{10,30},{64,30},{64,0},{70,0}}, color={0,0,0}));
  connect(wheelInertiaR.flange_b, flangeWheelRight)
    annotation (Line(points={{90,0},{100,0}}, color={0,0,0}));
  connect(fixWheelRight.frame_b, frameChassis) annotation (Line(
      points={{10,0},{0,0},{0,-100}},
      color={95,95,95},
      thickness=0.5));
  connect(fixWheelRight.frame_a, wheelRight.frame_a) annotation (Line(
      points={{30,0},{46,0}},
      color={95,95,95},
      thickness=0.5));
  connect(wheelLeft.heatPort,tyreLossRL. port_a) annotation (Line(points={{-40,-10},{-40,-40},{-56,-40}}, color={191,0,0}));
  connect(wheelRight.heatPort,tyreLossRR. port_a) annotation (Line(points={{40,-10},{40,-40},{56,-40}}, color={191,0,0}));
  connect(tyreLossRR.port_b, internalHeatPort) annotation (Line(points={{76,-40},{80,-40},{80,-60},{-100,-60},{-100,-80}},
        color={191,0,0}));
  connect(tyreLossRL.port_b, internalHeatPort) annotation (Line(points={{-76,-40},{-80,-40},{-80,-60},{-100,-60},{-100,-80}},
        color={191,0,0}));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{50,60},{50,60.1},{99.9,60.1}},
      color={255,204,51},
      thickness=0.5));
  connect(tyreLossRR.Q_flow, chassisBus.tyreLossRR) annotation (Line(points={{66,-29},{66,50},{50,50},{50,60}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
  connect(tyreLossRL.Q_flow, chassisBus.tyreLossRL) annotation (Line(points={{-66,-29},{-66,60},{50,60}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(points={{-40,-20},{-40,-44},{40,-44},{40,-20}}, color={135,135,135}),
        Line(points={{-50,20},{-30,20}}, color={135,135,135}),
        Line(points={{-50,-20},{-30,-20}}, color={135,135,135}),
        Line(points={{30,20},{50,20}}, color={135,135,135}),
        Line(points={{30,-20},{50,-20}}, color={135,135,135}),
        Line(points={{0,-44},{0,-60}}, color={135,135,135}),
        Line(
          points={{0,20},{0,102}},
          color={0,0,0},
          thickness=1),
        Rectangle(
          extent={{84,10},{14,-10}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Rectangle(
          extent={{20,20},{-20,-20}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={231,231,231}),
        Rectangle(
          extent={{8,14},{-20,-14}},
          fillPattern=FillPattern.Solid,
          fillColor={255,255,255},
          pattern=LinePattern.None),
        Rectangle(
          extent={{0,10},{-80,-10}},
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
          extent={{-18,78},{18,44}},
          lineColor={28,108,200},
          lineThickness=1,
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=220,
          closure=EllipseClosure.None),
        Line(
          points={{-20,72},{-14,72},{-12,66}},
          color={28,108,200},
          thickness=1),
        Text(
          extent={{-150,-60},{150,-100}},
          textColor={0,0,255},
          textString="%name")}),
    Documentation(info="<html>
<p>
An axle connecting rigidly the left and right wheels.
A&nbsp;driving torque can be applied by the connector <code>flangeMotor</code> which
is divided to the wheels by a&nbsp;differential. Optionally, the left and right torque
can be applied directly on the wheel flanges.
</p>
</html>"));
end SimpleAxisRear;
