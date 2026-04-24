within VDCWorkbenchModels.Examples.VehicleDrivetrains.VariantsVehicleDrivetrains;
model MiniAFMMotor
  extends BaseConfigurations.BaseMiniAFM;
  VehicleComponents.Powertrain.DrivetrainDifferentialIdeal powertrain(
    ratioEngineGear=dataPowertrain.ratioEngineGear,
    ratioFrontGear=dataPowertrain.ratioFrontGear,
    ratioRearGear=dataPowertrain.ratioRearGear) annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
  Data.MiniAFMPowertrain dataPowertrain annotation (Placement(transformation(extent={{0,80},{20,100}})));
equation
  connect(powertrain.flangeDriveFront, vehicle.flangeDriveFront)
    annotation (Line(points={{10,-26},{-20,-26},{-20,0},{-10,0}}, color={0,0,0}));
  connect(powertrain.flangeDriveRear, vehicle.flangeDriveRear)
    annotation (Line(points={{10,-34},{0,-34},{0,-10}}, color={0,0,0}));
  connect(powertrain.controlBus, controlBus) annotation (Line(
      points={{30,-30},{40,-30},{40,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
end MiniAFMMotor;
