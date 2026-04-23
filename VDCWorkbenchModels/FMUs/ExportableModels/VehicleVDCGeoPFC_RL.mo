within VDCWorkbenchModels.FMUs.ExportableModels;
model VehicleVDCGeoPFC_RL
  extends VehicleArchitectures.VDCWorkbench2025(
    redeclare VehicleComponents.Controllers.VDControl.VDCWorkbenchControl_RL controller);
  Modelica.Blocks.Interfaces.RealInput addRL_FrontSteering_in
    "Front steering angle to be superimposed"
    annotation (Placement(transformation(extent={{-142,38},{-102,78}})));
  Modelica.Blocks.Interfaces.RealInput addRL_TotalTorque_in
    "Total driving torque to be superimposed"
    annotation (Placement(transformation(extent={{-140,-10},{-100,30}})));
equation
  connect(controller.addRL_FrontSteering_in, addRL_FrontSteering_in)
    annotation (Line(points={{-42,58.2},{-46,58},{-122,58}}, color={0,0,127}));
  connect(controller.addRL_TotalTorque_in, addRL_TotalTorque_in) annotation (
      Line(points={{-42,54},{-94,54},{-94,10},{-120,10}}, color={0,0,127}));
end VehicleVDCGeoPFC_RL;
