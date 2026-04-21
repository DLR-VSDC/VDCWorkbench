within VDCWorkbenchModels.Data.BaseRecords;
record Drivetrain "Basic record containing template data of drive train"
  extends Modelica.Icons.Record;

  parameter String variantName= "" "Name of identification variant";
  parameter Real ratioEngineGear=48/16 "Ratio of gear on engine";
  parameter Real ratioFrontGear=43/11 "Ratio of front drive gear";
  parameter Real ratioRearGear=43/11 "Ratio of rear drive gear";

  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false),
      graphics={
        Text(
          extent={{-140,-100},{140,-130}},
          textColor={0,0,0},
          textString="%variantName")}),
    Diagram(coordinateSystem(preserveAspectRatio=false)));
end Drivetrain;
