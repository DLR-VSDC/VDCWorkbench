within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMGeoPFCTD_Racetrack
  extends VDCWorkbenchModels.VehicleArchitectures.MiniAFM(
    redeclare VehicleComponents.Controllers.VDControl.MiniAFMGeoPFCTD controller);
equation

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMGeoPFCTD_Racetrack;
