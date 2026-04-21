within VDCWorkbenchModels.Examples.VehicleDrivetrains;
model MiniAFMAccelerateSinusoidalSteering
  extends VariantsVehicleDrivetrains.MiniAFMMotor;

  Modelica.Blocks.Sources.Constant torque(k=0.005)
    annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
  Modelica.Blocks.Sources.Sine steeringWheelAngleDemand(
    startTime=0,
    amplitude=10,
    f=0.2)
    annotation (Placement(transformation(extent={{-100,70},{-80,90}})));
protected
  Modelica.Blocks.Math.UnitConversions.From_deg from_deg
    annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
  Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBus annotation (Placement(transformation(extent={{70,30},{90,50}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus1 annotation (Placement(transformation(extent={{70,50},{90,70}})));
equation
  connect(torque.y, electricMotorControlBus.torque) annotation (Line(
        points={{-39,40},{80,40}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(steeringWheelAngleDemand.y,from_deg. u)
    annotation (Line(points={{-79,80},{-62,80}}, color={0,0,127}));
  connect(from_deg.y, chassisControlBus1.steeringWheelAngle)
    annotation (
      Line(
        points={{-39,80},{-20,80},{-20,60},{80,60}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(controlBus.chassisControlBus, chassisControlBus1) annotation (Line(
      points={{100.1,-0.1},{100,-0.1},{100,60},{80,60}},
      color={255,204,51},
      thickness=0.5));
  connect(controlBus.electricMotorControlBus, electricMotorControlBus) annotation (Line(
      points={{100.1,-0.1},{100,-0.1},{100,40},{80,40}},
      color={255,204,51},
      thickness=0.5));

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMAccelerateSinusoidalSteering;
