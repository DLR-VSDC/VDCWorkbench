within VDCWorkbenchModels.Data.Tracks;
partial record BaseTrack "Basic record containing template data of tracks / paths"
  extends Modelica.Icons.Record;

  parameter String variantName = "Racetrack" "Name of track";
  parameter String filePath = ModelicaServices.ExternalReferences.loadResource(
    "modelica://VDCWorkbenchModels/Resources/Maps/Racetrack.mat")
    "File where path table pathName is stored" annotation (
      Dialog(
        loadSelector(
          filter="Matlab files(*.mat)",
          caption="Open data file")));
  parameter String pathName = "path" "Table name in filePath";
  parameter Boolean isClosed = false "= true, if path is a closed track";
  parameter Modelica.Units.SI.Position maxArcLength = 2.312560625428274e+03 "Maximum arc length value on path file";

  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false),
      graphics={
        Text(
          extent={{-140,-100},{140,-130}},
          textColor={0,0,0},
          textString="%variantName")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This record contains track data in order to facilitate track selection as used e.g. in the
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.TimeIndependetPathInterpolation.CoGTIPI\">time-independent path interpolation</a>.
</p>
</html>"));
end BaseTrack;
