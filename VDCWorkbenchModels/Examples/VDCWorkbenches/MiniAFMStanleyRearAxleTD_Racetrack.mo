within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMStanleyRearAxleTD_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.RearAxleStanleyControllerTD controller(
      redeclare VDCWorkbenchModels.Data.Tracks.RacetrackMini track));
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMStanleyRearAxleTD_Racetrack;
