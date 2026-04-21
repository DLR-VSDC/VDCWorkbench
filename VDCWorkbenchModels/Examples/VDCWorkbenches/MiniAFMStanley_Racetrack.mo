within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanley_Racetrack
  extends VehicleDrivetrains.VariantsVehicleDrivetrains.MiniAFMMotor;
  VehicleComponents.Controllers.VDControl.StanleyController stanleyController
    annotation (Placement(transformation(extent={{0,40},{20,60}})));
equation
  connect(stanleyController.controlBus, controlBus) annotation (Line(
      points={{10,40},{10,16},{68,16},{68,0},{100,0}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanley_Racetrack;
