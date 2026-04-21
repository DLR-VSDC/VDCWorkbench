within VDCWorkbenchModels.VehicleComponents.Powertrain;
model DrivetrainDifferentialIdeal "Powertrain of the miniAFM"
  extends Modelica.Thermal.HeatTransfer.Interfaces.PartialConditionalHeatPort(
    useHeatPort=false,
    T=298.15);

  parameter Real ratioEngineGear=48/16 "Ratio of gear on engine";
  parameter Real ratioFrontGear=43/11 "Ratio of front drive gear";
  parameter Real ratioRearGear=43/11 "Ratio of rear drive gear";

  Modelica.Mechanics.Rotational.Components.IdealGear engineGear(
    ratio=ratioEngineGear)
    annotation (Placement(transformation(extent={{0,-10},{-20,10}})));
  PlanarMechanics.VehicleComponents.DifferentialGear differentialGear
    annotation (Placement(transformation(extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-40,0})));
  Modelica.Mechanics.Rotational.Components.IdealGear frontGear(
    ratio=ratioFrontGear)
    annotation (Placement(transformation(extent={{-60,30},{-80,50}})));
  Modelica.Mechanics.Rotational.Components.IdealGear rearGear(
    ratio=ratioRearGear)
    annotation (Placement(transformation(extent={{-60,-50},{-80,-30}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDriveFront
    "Flange of front axle"
    annotation (Placement(transformation(extent={{-110,30},{-90,50}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeDriveRear
    "Flange of rear axle"
    annotation (Placement(transformation(extent={{-110,-50},{-90,-30}})));
  Modelica.Mechanics.Rotational.Sources.Torque torque
    annotation (Placement(transformation(extent={{30,-10},{10,10}})));
  Utilities.Interfaces.ControlBus controlBus annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={100,0})));
  Modelica.Thermal.HeatTransfer.Sources.FixedHeatFlow fixZeroHeatFlow(
    final Q_flow=0,
    final T_ref=T) if useHeatPort annotation (Placement(transformation(extent={{-20,-70},{-40,-50}})));
protected
  Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBus
    annotation (Placement(transformation(extent={{50,-10},{70,10}})));
equation

  connect(differentialGear.flange_b, engineGear.flange_b)
    annotation (Line(points={{-30,0},{-20,0}},color={0,0,0}));
  connect(differentialGear.flange_right, rearGear.flange_a)
    annotation (Line(points={{-40,-10},{-40,-40},{-60,-40}}, color={0,0,0}));
  connect(differentialGear.flange_left, frontGear.flange_a)
    annotation (Line(points={{-40,10},{-40,40},{-60,40}}, color={0,0,0}));
  connect(flangeDriveFront, frontGear.flange_b)
    annotation (Line(points={{-100,40},{-80,40}}, color={0,0,0}));
  connect(flangeDriveRear, rearGear.flange_b)
    annotation (Line(points={{-100,-40},{-80,-40}}, color={0,0,0}));
  connect(engineGear.flange_a, torque.flange)
    annotation (Line(points={{0,0},{10,0}}, color={0,0,0}));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus)
    annotation (Line(
      points={{99.9,0.1},{99.9,0},{60,0}},
      color={255,204,51},
      thickness=0.5));
  connect(torque.tau, electricMotorControlBus.torque) annotation (Line(points={{32,0},{60,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(fixZeroHeatFlow.port, internalHeatPort) annotation (Line(points={{-40,-60},{-100,-60},{-100,-80}}, color={191,0,0}));
  annotation (
    Icon(
      graphics={
        Rectangle(
          extent={{-8,6},{18,-6}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{18,28},{92,-26}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,128,255}),
        Rectangle(
          extent={{100,28},{92,-26}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={128,128,128}),
        Polygon(
          points={{14,-42},{28,-42},{40,-8},{72,-8},{88,-42},{102,-42},{102,-50},{
                14,-50},{14,-42}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,46},{-40,34}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-100,-34},{-40,-46}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-40,46},{-30,-46}},
          lineColor={0,0,0},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={95,95,95}),
        Ellipse(
          extent={{-46,-28},{-24,-50}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-48,50},{-26,28}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-58,26},{-8,-26}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Solid,
          fillColor={95,95,95},
          radius=10),
        Polygon(
          points={{-8,-30},{-8,-86},{-18,-86},{0,-106},{18,-86},{8,-86},{8,-30},{-8,-30}},
          lineColor={127,0,0},
          fillColor={127,0,0},
          fillPattern=FillPattern.Solid,
          visible=useHeatPort),
        Text(
          extent={{-150,110},{150,70}},
          textColor={0,0,255},
          textString="%name"),
        Line(
          points={{-32,0},{-70,-100},{-100,-100}},
          color={127,0,0},
          visible=useHeatPort)}),
    Documentation(
      info="<html>
<p>
Powertrain with three traction motors. One motor is to be connected to the drive flange
of the front vehicle axle. The remaining two motors drive rear axle wheels, each on one side.
</p>
</html>"));
end DrivetrainDifferentialIdeal;
