within VDCWorkbenchModels.Data;
record MiniAFMPowertrain "MiniAFM's drive train parameters"
  extends BaseRecords.Drivetrain(
    variantName="MiniAFM",
    ratioEngineGear=48/16,
    ratioFrontGear=43/11,
    ratioRearGear=43/11);

  annotation (
    Documentation(
      info="<html>
<p>
Set of parameters for the <em>MiniAFM&apos;s</em> drivetrain.
</p>
</html>"));
end MiniAFMPowertrain;
