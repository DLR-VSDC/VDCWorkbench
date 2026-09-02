within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMGeoPFC_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.MiniAFMGeoPFC controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMGeoPFC_Racetrack;
