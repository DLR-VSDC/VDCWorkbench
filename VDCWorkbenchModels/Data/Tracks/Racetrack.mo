within VDCWorkbenchModels.Data.Tracks;
record Racetrack "Racetrack by DLR"
  extends BaseTrack(
    variantName="Racetrack",
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/Racetrack.mat"),
    pathName="path",
    isClosed=true,
    maxArcLength=150);

  annotation (
    Documentation(
      info="<html>
<p>
A&nbsp; synthetic racetrack by DLR for use cases.
</p>
</html>"));
end Racetrack;
