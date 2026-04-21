within VDCWorkbenchModels.Examples.VehicleDrivetrains.VariantsVehicleDrivetrains;
model MiniAFMMotor
  extends BaseConfigurations.BaseMiniAFM;
  VehicleComponents.Powertrain.DrivetrainDifferentialIdeal powertrainMiniAFM(
    ratioEngineGear=miniAFMPowertrain.ratioEngineGear,
    ratioFrontGear=miniAFMPowertrain.ratioFrontGear,
    ratioRearGear=miniAFMPowertrain.ratioRearGear) annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
  Data.MiniAFMPowertrain miniAFMPowertrain
    annotation (Placement(transformation(extent={{0,80},{20,100}})));
equation
  connect(powertrainMiniAFM.flangeDriveFront, vehicle.flangeDriveFront)
    annotation (Line(points={{10,-26},{-20,-26},{-20,0},{-10,0}}, color={0,0,0}));
  connect(powertrainMiniAFM.flangeDriveRear, vehicle.flangeDriveRear)
    annotation (Line(points={{10,-34},{0,-34},{0,-10}},
        color={0,0,0}));
  connect(powertrainMiniAFM.controlBus, controlBus) annotation (Line(
      points={{30,-23},{40,-23},{40,0},{100,0}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
end MiniAFMMotor;
