within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyPredictiveTD_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.PredStanleyControllerTD controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanleyPredictiveTD_Racetrack;
