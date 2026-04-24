within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMGeoPFC_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.MiniAFMGeoPFC controller(
      redeclare VDCWorkbenchModels.Data.Tracks.RacetrackMini track));
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMGeoPFC_Racetrack;
