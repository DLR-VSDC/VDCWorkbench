within VDCWorkbenchModels.FMUs.Testbench;
model ResidualDRLforPFC_Racetrack
  extends Modelica.Icons.Example;
  ExportableModels.VehicleVDCGeoPFC_RL vDCGeoPFCforRL_Racetrack(
    path_yaw_init(displayUnit="rad"),
    v_scl(value=1.0)) annotation (Placement(transformation(extent={{-20,20},{20,60}})));
  DRLAgents.ResidualDRLgeoPFC residualDRLgeoPFC
    annotation (Placement(transformation(extent={{10,-20},{-10,0}})));
equation
  connect(residualDRLgeoPFC.fmuOutputsBus, vDCGeoPFCforRL_Racetrack.fmuOutputsBus) annotation (Line(
      points={{10,-10},{30,-10},{30,36},{20,36}},
      color={255,204,51},
      thickness=0.5));
  connect(residualDRLgeoPFC.residualDelta, vDCGeoPFCforRL_Racetrack.residualRL_steeringWheelAngle_in)
    annotation (Line(points={{-11,-4},{-40,-4},{-40,51.6},{-24.4,51.6}},
        color={0,0,127}));
  connect(residualDRLgeoPFC.residualTorque, vDCGeoPFCforRL_Racetrack.residualRL_torque_in)
    annotation (Line(points={{-11,-16},{-30,-16},{-30,42},{-24,42}}, color={0,
          0,127}));
  annotation (
    experiment(
      StopTime=500,
      Interval=0.05,
      __Dymola_Algorithm="Dassl"),
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)));
end ResidualDRLforPFC_Racetrack;
