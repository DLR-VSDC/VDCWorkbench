within VDCWorkbenchModels.Data.Tracks;
record Techlab2SBahn "Track from DLR's TechLab to S-Bahn (NonOpt) - for GeoPFC"
  extends BaseTrack(
    variantName="DLR TechLab to S-Bahn (NonOpt)",
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt.mat"),
    pathName="path",
    isClosed=false,
    maxArcLength=2.312560625428274e+03);

  annotation (
    Documentation(
      info="<html>
<p>
A&nbsp;route from the <em>TechLab</em> building at the DLR&apos;s site Oberpfaffenhofen to
the railway station in We&szlig;ling, Germany.
</p>
</html>"));
end Techlab2SBahn;
