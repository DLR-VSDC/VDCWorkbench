within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyRearAxle_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.RearAxleStanleyController controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanleyRearAxle_Racetrack;
