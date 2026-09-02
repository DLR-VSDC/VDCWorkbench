within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyTD_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.StanleyControllerTD controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanleyTD_Racetrack;
