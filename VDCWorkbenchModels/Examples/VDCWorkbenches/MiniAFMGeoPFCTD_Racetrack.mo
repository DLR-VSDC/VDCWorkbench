within VDCWorkbenchModels.Examples.VDCWorkbenches;
model MiniAFMGeoPFCTD_Racetrack
  extends VehicleDrivetrains.VariantsVehicleDrivetrains.MiniAFMMotor;
  VehicleComponents.Controllers.VDControl.MiniAFMGeoPFCTD miniAFM_TD_GeoPFC
    annotation (Placement(transformation(extent={{-2,40},{18,60}})));
equation
  connect(miniAFM_TD_GeoPFC.controlBus, controlBus) annotation (Line(
      points={{8,40},{8,16},{70,16},{70,0},{100,0}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));

  annotation (
    experiment(
      StopTime=9.5,
      __Dymola_Algorithm="Dassl"));
end MiniAFMGeoPFCTD_Racetrack;
