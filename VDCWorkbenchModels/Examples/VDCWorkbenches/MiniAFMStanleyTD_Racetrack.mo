within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyTD_Racetrack
  extends VehicleDrivetrains.VariantsVehicleDrivetrains.MiniAFMMotor;
  VehicleComponents.Controllers.VDControl.StanleyControllerTD stanleyController_TD
    annotation (Placement(transformation(extent={{20,40},{40,60}})));
equation
  connect(stanleyController_TD.controlBus, controlBus) annotation (Line(
      points={{30,40},{30,0},{100,0}},
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
end MiniAFMStanleyTD_Racetrack;
