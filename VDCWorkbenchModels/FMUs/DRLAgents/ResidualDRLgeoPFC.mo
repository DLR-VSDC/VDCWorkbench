within VDCWorkbenchModels.FMUs.DRLAgents;
model ResidualDRLgeoPFC
  extends Modelica.Blocks.Icons.Block;
  SMArtInt.Blocks.EvaluateSimpleFeedForwardNeuralNetwork
    evaluateSimpleFeedForwardNeuralNetwork(
    pathToTfLiteFile=ModelicaServices.ExternalReferences.loadResource(
      "modelica://VDCWorkbenchModels/Resources/DRL_Agents/residualDRLgeoPFC.onnx"),
    numberOfInputs=32,
    numberOfOutputs=2)
    annotation (Placement(transformation(extent={{8,-10},{28,10}})));
  Modelica.Blocks.Interfaces.RealOutput residualDelta "Residual steering action"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput residualTorque "Residual torque"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Utilities.Blocks.AssembleRLfeatureVec featureVecRL annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
  Utilities.Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-100,0})));
  Modelica.Clocked.RealSignals.Sampler.SampleClocked sample1[16]
    annotation (Placement(transformation(extent={{-54,-6},{-42,6}})));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicRealClock periodicClock(
    period=0.05,
    useSolver=false)
    annotation (Placement(transformation(extent={{-74,-46},{-62,-34}})));
  Modelica.Clocked.RealSignals.Sampler.Hold hold1[2]
    annotation (Placement(transformation(extent={{40,-6},{52,6}})));
  Modelica.Clocked.RealSignals.NonPeriodic.UnitDelay unitDelay1[16]
    annotation (Placement(transformation(extent={{-24,-26},{-12,-14}})));
  Modelica.Blocks.Math.Gain denormalize_steering(k=1.117010721276371)
    annotation (Placement(transformation(extent={{76,52},{92,68}})));
  Modelica.Blocks.Math.Gain denormalize_torque(k=150.0)
    annotation (Placement(transformation(extent={{78,-68},{94,-52}})));
equation
  connect(featureVecRL.fmuOutputsBus, fmuOutputsBus) annotation (Line(
      points={{-88,0},{-100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(featureVecRL.featureVec, sample1.u) annotation (
      Line(points={{-67,0},{-55.2,0}}, color={0,0,127}));
  connect(periodicClock.y, sample1[1].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[2].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[3].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[4].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[5].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[6].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[7].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[8].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[9].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[10].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[11].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[12].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[13].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[14].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[15].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[16].clock) annotation (Line(
      points={{-61.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(sample1.y, unitDelay1.u) annotation (Line(points={{-41.4,0},{-32,0},{
          -32,-20},{-25.2,-20}}, color={0,0,127}));
  connect(sample1[1:16].y, evaluateSimpleFeedForwardNeuralNetwork.arrayIn[1, 1:
    16]) annotation (Line(points={{-41.4,0},{8.2,0}}, color={0,0,127}));
  connect(evaluateSimpleFeedForwardNeuralNetwork.arrayOut[1, :], hold1.u)
    annotation (Line(points={{28,0},{38.8,0}}, color={0,0,127}));
  connect(unitDelay1[1:16].y, evaluateSimpleFeedForwardNeuralNetwork.arrayIn[1,
    17:32]) annotation (Line(points={{-11.4,-20},{0,-20},{0,0},{8.2,0}}, color=
          {0,0,127}));
  connect(hold1[1].y, denormalize_steering.u) annotation (Line(points={{52.6,0},
          {60,0},{60,60},{74.4,60}}, color={0,0,127}));
  connect(hold1[2].y, denormalize_torque.u) annotation (Line(points={{52.6,0},{
          60,0},{60,-60},{76.4,-60}}, color={0,0,127}));
  connect(denormalize_steering.y, residualDelta)
    annotation (Line(points={{92.8,60},{110,60}}, color={0,0,127}));
  connect(denormalize_torque.y, residualTorque)
    annotation (Line(points={{94.8,-60},{110,-60}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-42,42},{-38,38}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-42,2},{-38,-2}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-42,-38},{-38,-42}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,62},{2,58}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,22},{2,18}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,-18},{2,-22}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2,-58},{2,-62}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,42},{42,38}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,2},{42,-2}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,-38},{42,-42}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(points={{-80,0},{-100,0}}, color={28,108,200}),
        Line(points={{-80,0},{-40,40},{0,60},{40,40},{60,0}}, color={28,108,200}),
        Line(points={{-80,0},{-40,0},{0,20},{40,0},{60,0}}, color={28,108,200}),
        Line(points={{-80,0},{-40,-40},{0,-60},{40,-40},{60,0}}, color={28,108,200}),
        Line(points={{-40,0},{0,-20},{40,0}}, color={28,108,200}),
        Line(points={{-40,40},{0,20},{40,40}}, color={28,108,200}),
        Line(points={{-40,-40},{0,-20},{40,-40}}, color={28,108,200}),
        Line(points={{-40,40},{0,-20},{40,40}}, color={28,108,200}),
        Line(points={{-40,-40},{0,20},{40,-40}}, color={28,108,200}),
        Line(points={{-40,40},{0,-60},{40,40}}, color={28,108,200}),
        Line(points={{-40,-40},{0,60},{40,-40}}, color={28,108,200}),
        Line(points={{60,0},{70,0},{80,60},{100,60}}, color={0,0,127}),
        Line(points={{60,0},{70,0},{80,-60},{100,-60}}, color={0,0,127})}),
    Diagram(
      coordinateSystem(preserveAspectRatio=false)));
end ResidualDRLgeoPFC;
