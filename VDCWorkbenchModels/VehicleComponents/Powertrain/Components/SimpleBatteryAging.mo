within VDCWorkbenchModels.VehicleComponents.Powertrain.Components;
model SimpleBatteryAging
  "Simple battery model based on a inner resistance and SOC_OCV table and aging parameterization"

  parameter VDCWorkbenchModels.Utilities.Types.StateOfCharge SOC_init = 1 "Initial value of SOC"
    annotation (Dialog(group="Initialization"));

  parameter Modelica.Units.NonSI.ElectricCharge_Ah C_N=cellParameters.C_N
    "Nominal cell capacity @ C=0.25, T = 25C";
  parameter Real no_s=cellParameters.no_s "Number of cells connected in series";
  parameter Real no_p=cellParameters.no_p
    "Number of cells connected in parallel";

  parameter Boolean includeHeatPort = false "True, if heatPort is enabled";
  parameter Boolean includeGround = false
    "True, if negative electrical pin is internally grounded";
  parameter String SOC_OCV_table = "noName"
    "Path and file name of SOC_OCV table";
  parameter String Ri_Lookup_table = "noName" "Path and file name of Ri table";
  parameter Modelica.Units.SI.HeatCapacity Ch_cell=cellParameters.Ch_cell
    "Heat capacity of one cell";

  parameter Real N_cycles=cellParameters.N_cycles "Number of cell cycles";
  parameter Real C_rate=cellParameters.C_rate "C-Rate during aging";
  parameter Modelica.Units.NonSI.Temperature_degC T_aged=cellParameters.T_aged "Temperature during aging";
  parameter Real b[10]=cellParameters.b
    "Aging parameters";

  replaceable parameter Data.BaseRecords.Battery cellParameters
    constrainedby Data.BaseRecords.Battery "Set of common battery parameters"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

  //Real T_aged;
  Modelica.Blocks.Math.Division capacityAged2
    annotation (Placement(transformation(extent={{32,-26},{44,-14}})));
  Modelica.Thermal.HeatTransfer.Celsius.TemperatureSensor temperatureSensor1
    annotation (Placement(transformation(extent={{-48,-78},{-32,-62}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow resistorHeatflow if includeHeatPort
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-20,-16})));
  Modelica.Thermal.HeatTransfer.Sources.FixedTemperature enviTemp(T=298.15)
    "Environment temperature"
    annotation (Placement(transformation(extent={{-98,-98},{-82,-82}})));
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor thermalResistor(R=20)
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=0,
        origin={-70,-90})));
  AgingCalculation agingCalc(
    C_N=C_N,
    no_s=no_s,
    no_p=no_p,
    N_cycles=N_cycles,
    C_rate=C_rate,
    T_aged=T_aged,
    b=b)
    annotation (Placement(transformation(rotation=0, extent={{52,-60},{32,-40}})));

protected
  Modelica.Electrical.Analog.Basic.Ground ground if includeGround
  annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-100,-31})));
public
  Modelica.Electrical.Analog.Interfaces.PositivePin pin_p "Positive electrical pin"
    annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
  Modelica.Electrical.Analog.Interfaces.NegativePin pin_n "Negative electrical pin"
    annotation (Placement(transformation(extent={{-90,-70},{-110,-50}}), iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Blocks.Interfaces.RealOutput voltage "Total pack voltage"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput SOC "State of charge"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput current "Total pack current"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort if includeHeatPort
    "Conditional port for exhaust heat flow"
    annotation (Placement(transformation(extent={{-10,-110},{10,-90}},
          rotation=0)));
  Modelica.Electrical.Analog.Basic.VariableResistor variableResistor(
    useHeatPort=true)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,20})));
  Modelica.Electrical.Analog.Sensors.CurrentSensor sensorCurrent
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=-90,
        origin={-60,-30})));
  Modelica.Electrical.Analog.Sources.SignalVoltage cellVoltage
    "Voltage calculated by the mESC"
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=90,
        origin={-60,50})));
  Modelica.Blocks.Tables.CombiTable1Ds lookup_SOC_OCV(
    tableOnFile=false,
    table=cellParameters.SOC_OCV,
    tableName="SOC_OCV",
    fileName=SOC_OCV_table,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    "Open circuit voltage dependent state of charge"
    annotation (Placement(transformation(extent={{60,50},{40,70}})));
  Modelica.Blocks.Tables.CombiTable1Ds lookup_Ri(
    tableOnFile=false,
    table=cellParameters.R_i,
    tableName="R_i_Mod",
    fileName=Ri_Lookup_table,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint)
    "Dynamic R_i"
    annotation (Placement(transformation(extent={{60,10},{40,30}})));
  Modelica.Blocks.Continuous.LimIntegrator integratorSOC(
    outMax=1,
    outMin=0,
    y_start=SOC_init,
    k=1)
    annotation (Placement(transformation(extent={{54,-30},{74,-10}})));
  Modelica.Blocks.Math.Gain packScale(k=no_s/no_p)
    annotation (Placement(transformation(extent={{20,10},{0,30}})));
  Modelica.Blocks.Math.Gain voltageScale(k=no_s)
    annotation (Placement(transformation(extent={{20,50},{0,70}})));
  Modelica.Electrical.Analog.Sensors.VoltageSensor voltageSensor annotation (
      Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-90,40})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor cellThermCap(
    C=Ch_cell*no_s*no_p,
    T(start=298.15, fixed=true)) "Cell heat capacitor"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-40,-90})));
  Utilities.Interfaces.ControlBus controlBus annotation (Placement(
        transformation(extent={{40,-120},{80,-80}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor heatFlowSensor
    annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=270,
        origin={-76,0})));

protected
  VehicleInterfaces.Interfaces.BatteryBus batteryBus annotation (Placement(transformation(extent={{-10,-60},{10,-40}}), iconTransformation(extent={{-16,-76},{4,-56}})));
equation
  connect(ground.p, pin_n) annotation (Line(
      points={{-90,-31},{-90,-60},{-100,-60}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(cellVoltage.p, pin_p) annotation (Line(
      points={{-60,60},{-100,60}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(cellVoltage.n,variableResistor. p) annotation (Line(
      points={{-60,40},{-60,30}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(variableResistor.n,sensorCurrent. n) annotation (Line(
      points={{-60,10},{-60,-20}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(integratorSOC.y,lookup_SOC_OCV. u) annotation (Line(
      points={{75,-20},{80,-20},{80,60},{62,60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(integratorSOC.y, SOC) annotation (Line(
      points={{75,-20},{80,-20},{80,0},{110,0}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(current,sensorCurrent. i) annotation (Line(
      points={{110,-60},{80,-60},{80,-74},{10,-74},{10,-30},{-49,-30}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(packScale.y,variableResistor. R) annotation (Line(
      points={{-1,20},{-48,20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(voltageScale.u,lookup_SOC_OCV. y[1]) annotation (Line(
      points={{22,60},{39,60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(voltageScale.y,cellVoltage. v) annotation (Line(
      points={{-1,60},{-10,60},{-10,50},{-48,50}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(voltageSensor.v, voltage) annotation (Line(
      points={{-79,40},{-72,40},{-72,80},{90,80},{90,60},{110,60}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(lookup_Ri.u, lookup_SOC_OCV.u) annotation (Line(points={{62,20},{80,20},{80,60},{62,60}},
                            color={0,0,127}));
  connect(lookup_Ri.y[1], packScale.u)
    annotation (Line(points={{39,20},{22,20}},color={0,0,127}));
  connect(pin_n, sensorCurrent.p) annotation (
      Line(points={{-100,-60},{-60,-60},{-60,-40}}, color={0,0,255}));
  connect(voltageSensor.p, pin_p) annotation (
      Line(points={{-90,50},{-90,60},{-100,60}}, color={0,0,255}));
  connect(voltageSensor.n, pin_n) annotation (
      Line(points={{-90,30},{-90,-60},{-100,-60}}, color={0,0,255}));
  connect(batteryBus, controlBus.batteryBus) annotation (Line(
      points={{0,-50},{0,-80},{60,-80},{60,-99.9},{60.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(integratorSOC.y, batteryBus.SOC) annotation (Line(points={{75,-20},{80,-20},{80,0},{2,0},{2,-50},{0,-50}}, color={0,0,127}), Text(
      string="%second",
      index=4,
      extent={{3,3},{3,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensorCurrent.i, batteryBus.current)
    annotation (Line(points={{-49,-30},{10,-30},{10,-48},{0,-48},{0,-50}},
      color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(heatFlowSensor.Q_flow, batteryBus.Q_flow) annotation (Line(points={{-65,0},{-4,0},{-4,-50},{0,-50}}, color={0,0,127}), Text(
      string="%second",
      index=2,
      extent={{-3,3},{-3,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(integratorSOC.y, agingCalc.SoC) annotation (Line(points={{75,-20},{80,
          -20},{80,-44},{54,-44}}, color={0,0,127}));
  connect(capacityAged2.y, integratorSOC. u)
    annotation (Line(points={{44.6,-20},{52,-20}}, color={0,0,127}));
  connect(capacityAged2.u1, sensorCurrent.i) annotation (Line(points={{30.8,-16.4},{10,-16.4},{10,-30},{-49,-30}},
        color={0,0,127}));
  connect(temperatureSensor1.port, cellThermCap.port) annotation (Line(points={{-48,-70},{-56,-70},{-56,-90},{-50,-90}},
        color={191,0,0}));
  connect(temperatureSensor1.T, batteryBus.TemperatureBattery) annotation (Line(points={{-32,-70},{-2,-70},{-2,-50},{0,-50}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,-3},{6,-3}},
      horizontalAlignment=TextAlignment.Left));
  connect(variableResistor.heatPort, heatFlowSensor.port_a) annotation (Line(
        points={{-70,20},{-76,20},{-76,10}}, color={191,0,0}));
  connect(heatFlowSensor.port_b, cellThermCap.port)
    annotation (Line(points={{-76,-10},{-76,-56},{-56,-56},{-56,-90},{-50,-90}},
        color={191,0,0}));
  connect(resistorHeatflow.port, heatPort) annotation (Line(points={{-20,-26},{-20,-100},{0,-100}},
        color={191,0,0}));
  connect(enviTemp.port, thermalResistor.port_a)
    annotation (Line(points={{-82,-90},{-78,-90}}, color={191,0,0}));
  connect(sensorCurrent.i, agingCalc.current) annotation (Line(points={{-49,-30},{10,-30},{10,-74},{80,-74},{80,-50},{54,-50}},
        color={0,0,127}));
  connect(temperatureSensor1.T, agingCalc.temperature) annotation (Line(points={{-32,-70},{70,-70},{70,-56},{54,-56}},
        color={0,0,127}));
  connect(heatFlowSensor.Q_flow, resistorHeatflow.Q_flow) annotation (Line(points={{-65,0},{-20,0},{-20,-6}}, color={0,0,127}));
  connect(thermalResistor.port_b, cellThermCap.port)
    annotation (Line(points={{-62,-90},{-50,-90}}, color={191,0,0}));
  connect(capacityAged2.u2, agingCalc.Capacity) annotation (Line(points={{30.8,-23.6},{20,-23.6},{20,-44},{31,-44}},
        color={0,0,127}));
  connect(agingCalc.DeltaC_dN, batteryBus.DeltaC_dN) annotation (Line(points={{31,-56},{20,-56},{20,-52},{0,-52},{0,-50}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(agingCalc.C_aged, batteryBus.C_aged) annotation (Line(points={{31,-50},{0,-50}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(voltageScale.y, batteryBus.OCV) annotation (Line(points={{-1,60},{-10,60},{-10,8},{0,8},{0,-50}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(voltageSensor.v, batteryBus.voltage) annotation (Line(points={{-79,40},{-72,40},{-72,36},{-12,36},{-12,6},{-2,6},{-2,-50},{0,-50}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  annotation (
    Icon(graphics={
        Text(
          extent={{-90,0},{110,-30}},
          textColor={0,0,0},
          textString="%no_s%s %no_p%pin_p"),
        Line(
          points={{-100,-60},{-100,-96},{-100,-108}},
          color={0,0,255},
          smooth=Smooth.None,
          visible=includeGround),
        Line(
          points={{-120,-108},{-80,-108}},
          color={0,0,255},
          smooth=Smooth.None,
          visible=includeGround),
        Line(
          points={{-112,-116},{-88,-116}},
          color={0,0,255},
          smooth=Smooth.None,
          visible=includeGround),
        Line(
          points={{-106,-124},{-94,-124}},
          color={0,0,255},
          smooth=Smooth.None,
          visible=includeGround),
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{70,34},{-36,18},{-36,-60},{70,-44},{70,34}},
          lineColor={85,85,255},
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-36,18},{-70,44},{-70,-34},{-36,-60},{-36,18}},
          lineColor={85,85,255},
          fillColor={215,236,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-70,44},{36,60},{70,34},{-36,18},{-70,44}},
          lineColor={85,85,255},
          fillColor={215,236,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-52,34},{-40,30}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-52,38},{-40,32}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-52,40},{-40,36}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,48},{50,44}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{38,52},{50,46}},
          lineColor={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{38,54},{50,50}},
          lineColor={0,0,0},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-38,38},{-28,34}},
          lineColor={0,0,0},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-22,40},{-12,36}},
          lineColor={0,0,0},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-8,42},{2,38}},
          lineColor={0,0,0},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{8,44},{18,40}},
          lineColor={0,0,0},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,46},{34,42}},
          lineColor={0,0,0},
          fillColor={0,255,0},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={255,0,0}),
        Rectangle(
          extent={{80,6},{100,-6}},
          fillPattern=FillPattern.Solid,
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{-100,6},{-80,-6}},
          fillPattern=FillPattern.Solid,
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=90,
          origin={52,-120},
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=45,
          origin={80,6},
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=135,
          origin={-6,80},
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=135,
          origin={122,-48},
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=45,
          origin={-50,-120},
          lineColor={95,95,95},
          fillColor={95,95,95}),
        Rectangle(
          extent={{20,58},{40,46}},
          fillPattern=FillPattern.Solid,
          rotation=90,
          origin={52,60},
          pattern=LinePattern.None,
          fillColor={95,95,95}),
        Ellipse(
          extent={{10,-10},{-10,10}},
          lineColor={95,95,95},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-62,4},{0,0}},
          color={95,95,95},
          thickness=1),
        Line(
          points={{0,0},{54,54}},
          color={95,95,95},
          thickness=1)}), Documentation(info="<html>
<p>
A functional battery with an equivalent electrical circuit model to model obtain
a&nbsp;good tradeoff between simulation speed and modeling accuracy.
</p>
<p>
The electrical circuit consists of an ideal voltage source in series with an internal
resistance, thus forming the terminal voltage of the cell. Both the internal voltage
and resistance depend on the battery state of charge <code>SOC</code>, a&nbsp;normalized
indicator for the amount of charge stored in the battery.
Lookup tables <code>SOC_OCV</code> and <code>R_i</code> are employed, respectively,
to characterize this variation, but are limited to room temperature values.
</p>
<p>
The model also includes an
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Powertrain.Components.AgingCalculation\">aging model</a>
<code>agingCalc</code> for predicting the capacity degradation of the battery due to
the charge/discharge events.
</p>
<p>
Additionally, there is a&nbsp;simple thermal model that captures heat flow between the battery
cell and a&nbsp;(constant-temperature) environment using a thermal resistor.
</p>
</html>"));
end SimpleBatteryAging;
