within VDCWorkbenchModels.Data.Tracks;
record Techlab2SBahnTIPI "Track from DLR's TechLab to S-Bahn (NonOpt)"
  extends BaseTrack(
    variantName="DLR TechLab to S-Bahn (NonOpt)",
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt_TIPI.mat"),
    pathName="path_TIPI",
    isClosed=false,
    maxArcLength=2.312560625428274e+03);

  annotation (
    Documentation(
      info="<html>
<p>
A&nbsp;route from the <em>TechLab</em> building at the DLR&apos;s site Oberpfaffenhofen to
the railway station in We&szlig;ling, Germany.
This definition is aimed to be used for models which utilize
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.TimeIndependetPathInterpolation\">time-independent path interpolation</a>.
</p>
</html>"));
end Techlab2SBahnTIPI;
