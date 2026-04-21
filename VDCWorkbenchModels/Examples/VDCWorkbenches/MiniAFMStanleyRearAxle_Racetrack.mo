within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyRearAxle_Racetrack
  extends VehicleDrivetrains.VariantsVehicleDrivetrains.MiniAFMMotor;
  VehicleComponents.Controllers.VDControl.RearAxleStanleyController rearAxleStanleyController
    annotation (Placement(transformation(extent={{0,40},{20,60}})));
equation
  connect(rearAxleStanleyController.controlBus, controlBus) annotation (Line(
      points={{10,40},{10,16},{70,16},{70,0},{100,0}},
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
end MiniAFMStanleyRearAxle_Racetrack;
