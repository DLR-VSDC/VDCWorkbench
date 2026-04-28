within VDCWorkbenchModels.VehicleComponents.Chassis;
model PlanarVehicle "Simple planar model of vehicle"
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialConditionalHeatPort;

  parameter Modelica.Units.SI.Velocity v_Start=0 "Initial velocity";

  parameter Modelica.Units.SI.Length trackWidth=trackWidth "Track width of vehicle";
  parameter Modelica.Units.SI.Length wheelBase=wheelBase "Wheelbase of Vehicle";
  parameter Modelica.Units.SI.Inertia Jz=Jz "Yaw inertia of vehicle";
  parameter Modelica.Units.SI.Mass m=m "Total mass of vehicle";
  parameter Modelica.Units.SI.Inertia J_steer=J_steer
    "Inertia of steering rack pinion mechanism";
  parameter Modelica.Units.SI.Length s[2]=s "Driving direction of the vehicle at angle phi = 0";
  parameter Modelica.Units.SI.Length R0=R0 "Radius of the wheel" annotation (Dialog(group="Wheels and tires"));
  parameter Modelica.Units.SI.Velocity vAdhesion=vAdhesion "Adhesion velocity" annotation (Dialog(group="Wheels and tires"));
  parameter Modelica.Units.SI.Velocity vSlide=vSlide "Sliding velocity" annotation (Dialog(group="Wheels and tires"));
  parameter Real mu_A=mu_A "Friction coefficient at adhesion" annotation (Dialog(group="Wheels and tires"));
  parameter Real mu_S=mu_S "Friction coefficient at sliding" annotation (Dialog(group="Wheels and tires"));
  parameter Modelica.Units.SI.Inertia J_wheel=J_wheel "Inertia of wheel" annotation (Dialog(group="Wheels and tires"));
  parameter Modelica.Units.SI.Force N=N "Normal force" annotation (Dialog(group="Wheels and tires"));
  parameter Real c_W=c_W "Drag coefficient" annotation (Dialog(group="Air resistance"));
  parameter Modelica.Units.SI.Area area=area
    "Frontal cross area of vehicle" annotation (Dialog(group="Air resistance"));
  parameter Modelica.Units.SI.Density rho=1.18 "Air density" annotation (Dialog(group="Air resistance"));
  parameter Modelica.Units.SI.Length r[2]=s
    "Driving direction of vehicle at angle frame_a.phi = 0" annotation (Dialog(group="Air resistance"));

  Components.CarBody carBody(
    wheelBase=wheelBase,
    Jz=Jz,
    m=m,
    v_long=v_Start) annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Components.SimpleAxisFront axleFront(
    final useHeatPort=useHeatPort,
    final T=T,
    R0=R0,
    s=s,
    N=N,
    vAdhesion=vAdhesion,
    vSlide=vSlide,
    mu_A=mu_A,
    mu_S=mu_S,
    J_wheel=J_wheel,
    J_steer=J_steer,
    v_long=v_Start,
    trackWidth=trackWidth,
    animate=carBody.animate,
    inertiaSteering(stateSelect=StateSelect.prefer))
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  Components.SimpleAxisRear axleRear(
    final useHeatPort=useHeatPort,
    final T=T,
    R0=R0,
    s=s,
    vAdhesion=vAdhesion,
    vSlide=vSlide,
    mu_A=mu_A,
    mu_S=mu_S,
    J_wheel=J_wheel,
    N=N,
    v_long=v_Start,
    trackWidth=trackWidth,
    animate=carBody.animate) annotation (Placement(transformation(extent={{-10,-30},{10,-50}})));
  PlanarMechanics.VehicleComponents.AirResistanceLongitudinal airResistanceLongitudinal(
    c_W=c_W,
    area=area,
    rho=rho,
    r=r) annotation (Placement(transformation(extent={{-30,10},{-50,30}})));
  Modelica.Mechanics.Rotational.Sources.Position positionSteeringWheel(w(fixed=true)) annotation (Placement(transformation(extent={{30,60},{10,80}})));
  Utilities.Interfaces.ControlBus controlBus annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=90,
        origin={100,80})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeWheelFL "Flange of front left wheel" annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeWheelFR "Flange of front right wheel" annotation (Placement(transformation(extent={{90,30},{110,50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeWheelRL "Flange of rear left wheel" annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a flangeWheelRR "Flange of rear right wheel" annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDriveRear "Flange of rear drive" annotation (Placement(transformation(extent={{-10,-110},{10,-90}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDriveFront "Flange of front drive" annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
protected
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{50,70},{70,90}}), iconTransformation(extent={{0,20},{20,40}})));
equation
  connect(carBody.frameAxleFront, axleFront.frameChassis) annotation (Line(
      points={{0,10},{0,30}},
      color={95,95,95},
      thickness=0.5));
  connect(axleRear.frameChassis, carBody.frameAxleRear) annotation (Line(
      points={{0,-30},{0,-10}},
      color={95,95,95},
      thickness=0.5));
  connect(carBody.controlBus, controlBus) annotation (Line(
      points={{10,6},{20,6},{20,46},{70,46},{70,80},{100,80}},
      color={255,204,51},
      thickness=0.5));
  connect(axleFront.controlBus, controlBus) annotation (Line(
      points={{10,46},{70,46},{70,80},{100,80}},
      color={255,204,51},
      thickness=0.5));
  connect(axleRear.controlBus, controlBus) annotation (Line(
      points={{10,-46},{20,-46},{20,46},{70,46},{70,80},{100,80}},
      color={255,204,51},
      thickness=0.5));
  connect(axleFront.flangeWheelLeft, flangeWheelFL) annotation (Line(points={{-10,40},{-100,40}}, color={0,0,0}));
  connect(axleFront.flangeWheelRight, flangeWheelFR) annotation (Line(points={{10,40},{100,40}}, color={0,0,0}));
  connect(axleRear.flangeWheelLeft, flangeWheelRL) annotation (Line(points={{-10,-40},{-100,-40},{-100,-60}},
        color={0,0,0}));
  connect(axleRear.flangeWheelRight, flangeWheelRR) annotation (Line(points={{10,-40},{100,-40},{100,-60}},
        color={0,0,0}));
  connect(axleRear.flangeDifferential, flangeDriveRear) annotation (Line(points={{0,-50},{0,-100}}, color={0,0,0}));
  connect(airResistanceLongitudinal.frame_a, carBody.frameAxleFront) annotation (Line(
      points={{-30,20},{0,20},{0,10}},
      color={95,95,95},
      thickness=0.5));
  connect(axleFront.heatPort, internalHeatPort) annotation (Line(points={{-10,30},{-20,30},{-20,-60},{-90,-60},{-90,-80},{-100,-80}}, color={191,0,0}));
  connect(axleRear.heatPort, internalHeatPort) annotation (Line(points={{-10,-30},{-20,-30},{-20,-60},{-90,-60},{-90,-80},{-100,-80}}, color={191,0,0}));
  connect(axleFront.flangeDifferential, flangeDriveFront) annotation (Line(points={{-4,50},{-60,50},{-60,0},{-100,0}}, color={0,0,0}));
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{60,80},{60,80.1},{99.9,80.1}},
      color={255,204,51},
      thickness=0.5));
  connect(positionSteeringWheel.phi_ref, chassisControlBus.steeringWheelAngle) annotation (Line(points={{32,70},{60,70},{60,80}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(positionSteeringWheel.flange, axleFront.flangeSteeringWheel) annotation (Line(points={{10,70},{4,70},{4,50},{3.8,50}}, color={0,0,0}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false),
      graphics={
        Line(points={{0,-100},{0,-70}}, color={0,0,0}),
        Line(points={{-100,0},{-40,0}}, color={0,0,0}),
        Rectangle(lineColor={64,64,64},
          fillColor={192,192,192},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,-56},{100,-64}}),
        Rectangle(lineColor={64,64,64},
          fillColor={192,192,192},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-100,44},{100,36}}),
        Rectangle(
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{70,20},{50,60}},
          radius=5),
        Rectangle(
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-50,-80},{-70,-40}},
          radius=5),
        Rectangle(
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{70,-80},{50,-40}},
          radius=5),
        Rectangle(
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.HorizontalCylinder,
          extent={{-50,20},{-70,60}},
          radius=5),
        Text(
          extent={{-150,140},{150,100}},
          textColor={0,0,255},
          textString="%name"),
        Polygon(
          points={{-50,90},{-60,40},{-60,-60},{-50,-84},{50,-84},{60,-60},{60,40},{50,90},{-50,90}},
          lineColor={28,108,200},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-50,50},{-40,10},{40,10},{50,50},{-50,50}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-34,-56},{-40,-74},{40,-74},{34,-56},{-34,-56}},
          lineColor={28,108,200},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-40,64},{0,36}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Solid,
          startAngle=0,
          endAngle=180,
          closure=EllipseClosure.None)}),
    Documentation(info="<html>
<p>
Planar vehicle containing vehicle&apos;s body, front and rear axle and air resistance.
The driving and braking torque can be applied in several ways &ndash; either individually
on wheel flanges or per axle. In the latter case, the torque is distributed to the axle&nbsp;s
wheels by a&nbsp;differentials.
Basic measurements of vehicle&apos;s planar dynamic are placed on the chassis bus.
</p>
<p>
The steering angle of the front wheels shall be given by the signal
<code>steeringWheelAngle</code> provided on the <code>chassisControlBus</code>.
</p>
</html>"));
end PlanarVehicle;
