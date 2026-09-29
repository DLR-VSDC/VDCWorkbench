within VDCWorkbenchModels.Data.Tracks;
record RacetrackMini "Racetrack by DLR for miniAFM"
  extends BaseTrack(
    variantName="Racetrack Mini",
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat"),
    pathName="path",
    isClosed=true,
    maxArcLength=22.737000000000002);

  annotation (
    Documentation(
      info="<html>
<p>
A&nbsp;scaled synthetic racetrack by DLR for use cases.
</p>
</html>"));
end RacetrackMini;
