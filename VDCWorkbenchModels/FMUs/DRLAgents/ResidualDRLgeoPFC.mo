within VDCWorkbenchModels.FMUs.DRLAgents;
model ResidualDRLgeoPFC
  extends Modelica.Blocks.Icons.Block;

protected
  constant Integer nIn = 16 "Number of inputs";

public
  SMArtInt.Blocks.EvaluateSimpleFeedForwardNeuralNetwork
    evaluateSimpleFeedForwardNeuralNetwork(
    pathToTfLiteFile=ModelicaServices.ExternalReferences.loadResource(
      "modelica://VDCWorkbenchModels/Resources/DRL_Agents/residualDRLgeoPFC.onnx"),
    numberOfInputs=32,
    numberOfOutputs=2)
    annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  Modelica.Blocks.Interfaces.RealOutput residualDelta "Residual steering action"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput residualTorque "Residual torque"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Utilities.Blocks.AssembleRLfeatureVec featureVecRL(
    featMax={6.981317007977318,640.0,6.981317007977318,640.0,2.5,10.0,0.7853981633974483,0.1,50.0,1.117010721276371,150.0,50.0,10.0,4.0,0.17453292519943295,1.0},
    featNominal={0.6981317007977318,400.0,0.6981317007977318,400.0,0.5,3.0,0.08726646259971647,0.02,20.0,1.117010721276371,150.0,20.0,3.0,1.5,0.017453292519943295,0.15})
    annotation (Placement(transformation(extent={{-88,-10},{-68,10}})));
  Utilities.Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-100,0})));
  Modelica.Clocked.RealSignals.Sampler.SampleClocked sample1[nIn]
    annotation (Placement(transformation(extent={{-54,-6},{-42,6}})));
  Modelica.Clocked.ClockSignals.Clocks.PeriodicRealClock periodicClock(
    period=0.05,
    useSolver=false)
    annotation (Placement(transformation(extent={{-84,-46},{-72,-34}})));
  Modelica.Clocked.RealSignals.Sampler.Hold hold1[2]
    annotation (Placement(transformation(extent={{50,-6},{62,6}})));
  Modelica.Clocked.RealSignals.NonPeriodic.UnitDelay unitDelay1[nIn]
    annotation (Placement(transformation(extent={{-26,-26},{-14,-14}})));
  Modelica.Blocks.Math.Gain denormalize_steering(k=1.117010721276371)
    annotation (Placement(transformation(extent={{76,52},{92,68}})));
  Modelica.Blocks.Math.Gain denormalize_torque(k=150.0)
    annotation (Placement(transformation(extent={{78,-68},{94,-52}})));
  Modelica.Blocks.Routing.Multiplex2 multiplex2(n1=nIn, n2=nIn) annotation (Placement(transformation(extent={{0,-6},{12,6}})));
equation
  connect(featureVecRL.fmuOutputsBus, fmuOutputsBus) annotation (Line(
      points={{-88,0},{-100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(featureVecRL.featureVec, sample1.u) annotation (
      Line(points={{-67,0},{-55.2,0}}, color={0,0,127}));
  connect(periodicClock.y, sample1[1].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[2].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[3].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[4].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[5].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[6].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[7].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[8].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[9].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[10].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[11].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[12].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[13].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[14].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[15].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(periodicClock.y, sample1[16].clock) annotation (Line(
      points={{-71.4,-40},{-48,-40},{-48,-7.2}},
      color={175,175,175},
      pattern=LinePattern.Dot,
      thickness=0.5));
  connect(sample1.y, unitDelay1.u) annotation (Line(points={{-41.4,0},{-32,0},{-32,-20},{-27.2,-20}},
        color={0,0,127}));
  connect(evaluateSimpleFeedForwardNeuralNetwork.arrayOut[1, :], hold1.u)
    annotation (Line(points={{40,0},{48.8,0}}, color={0,0,127}));
  connect(hold1[1].y, denormalize_steering.u) annotation (Line(points={{62.6,0},{70,0},{70,60},{74.4,60}},
        color={0,0,127}));
  connect(hold1[2].y, denormalize_torque.u) annotation (Line(points={{62.6,0},{70,0},{70,-60},{76.4,-60}},
        color={0,0,127}));
  connect(denormalize_steering.y, residualDelta)
    annotation (Line(points={{92.8,60},{110,60}}, color={0,0,127}));
  connect(denormalize_torque.y, residualTorque)
    annotation (Line(points={{94.8,-60},{110,-60}}, color={0,0,127}));
  connect(sample1.y, multiplex2.u1) annotation (Line(points={{-41.4,0},{-32,0},{-32,3.6},{-1.2,3.6}},
        color={0,0,127}));
  connect(unitDelay1.y, multiplex2.u2) annotation (Line(points={{-13.4,-20},{-10,-20},{-10,-3.6},{-1.2,-3.6}},
        color={0,0,127}));
  connect(multiplex2.y, evaluateSimpleFeedForwardNeuralNetwork.arrayIn[1, :]) annotation (Line(points={{12.6,0},{20.2,0}},         color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false),
      graphics={
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
