within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanley_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.StanleyController controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanley_Racetrack;
