within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.BaseClasses;
model BaseSubBusses "Base class for controllers with sub-busses"
  extends VehicleInterfaces.Icons.Controller;

  Utilities.Interfaces.ControlBus controlBus
    annotation (Placement(transformation(extent={{-20,-120},{20,-80}})));
  Modelica.Blocks.Sources.Constant const(k=1.0)
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
protected
  Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBus
    annotation (Placement(transformation(extent={{70,-30},{90,-10}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{70,10},{90,30}})));
equation
  connect(electricMotorControlBus, controlBus.electricMotorControlBus)
    annotation (Line(
      points={{80,-20},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{80,20},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=true)),
    Diagram(coordinateSystem(preserveAspectRatio=true)));
end BaseSubBusses;
