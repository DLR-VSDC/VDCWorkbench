within VDCWorkbenchModels.Utilities.Interfaces;
expandable connector ControlBus
  "Bus of VDCWorkbenchModels.Utilities.Interfaces: Minimal standard control bus"
  extends Modelica.Icons.SignalBus;

  VehicleInterfaces.Interfaces.AccessoriesBus accessoriesBus "Accessories bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.AccessoriesControlBus accessoriesControlBus
    "Accessories control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.BatteryBus batteryBus "Battery bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.BrakesBus brakesBus "Brakes bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.BrakesControlBus brakesControlBus
    "Brakes control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus "Chassis bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    "Chassis control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.DrivelineBus drivelineBus "Driveline bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.DrivelineControlBus drivelineControlBus
    "Driveline control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.DriverBus driverBus "Driver bus"
    annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorBus "Electric motor bus" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorBusFM "Bus of electric drive in front middle" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorBusRL "Bus of electric drive rear left" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorBusRR "Bus of electric drive rear right" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorControlBusFM "Control bus of electric drive in front middle" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorControlBusRL "Control bus of electric drive rear left" annotation (Dialog(enable=false));
  ElectricDriveBus electricMotorControlBusRR "Control bus of electric drive rear right" annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.ElectricMotorControlBus electricMotorControlBus
    "Electric motor control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.EngineBus engineBus "Engine bus"
    annotation (Dialog(enable=false));
  VehicleInterfaces.Interfaces.EngineControlBus engineControlBus
    "Engine control bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.TransmissionBus transmissionBus
    "Transmission bus" annotation(Dialog(enable=false));
  VehicleInterfaces.Interfaces.TransmissionControlBus transmissionControlBus
    "Transmission control bus" annotation(Dialog(enable=false));
  RexBus rexBus "Range extender (H2) bus" annotation(Dialog(enable=false));
  RefBus refBus "Reference signals for bWMS" annotation(Dialog(enable=false));
  EMAControls eMAControls "Control variables for EMA" annotation(Dialog(enable=false));
  ScoreBus scoreBus "Bus connector used for score values of the EMA" annotation(Dialog(enable=false));
  MotionDemandBus motionDemandBus "Bus connector used for PFC and CA vehicle control" annotation(Dialog(enable=false));

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}), graphics={
    	  Rectangle(
          extent={{-20,2},{20,-2}},
          lineColor={255,204,51},
          lineThickness=0.5)}),
    Documentation(
      info="<html>
<p>An empty expandable connector used as the top-level control signal bus in VehicleInterfaces.</p>
</html>"));
end ControlBus;
