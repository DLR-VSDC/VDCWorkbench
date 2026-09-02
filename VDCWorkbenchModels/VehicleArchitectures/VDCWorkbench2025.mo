within VDCWorkbenchModels.VehicleArchitectures;
partial model VDCWorkbench2025 "Vehicle architecture for Motor Vehicles Challenge"
  extends VehicleArchitectures.BaseArchitecture(
    v_Start=1, vehicle(useHeatPort=true,
      axleRear(
        wheelRight(phi_roll(fixed=true, start=0), w_roll(fixed=true))),
      axleFront(
        wheelRight(phi_roll(fixed=true, start=0), w_roll(fixed=true)))));
  extends Modelica.Blocks.Icons.Block;

  Modelica.Blocks.Interfaces.RealOutput Ploss(unit="W") "Instantaneous power loss"
    annotation (Placement(transformation(extent={{100,-80},{120,-60}})));
  Modelica.Blocks.Interfaces.RealOutput Eloss(unit="J") "Energy loss"
    annotation (Placement(transformation(extent={{100,-60},{120,-40}})));
  Modelica.Thermal.HeatTransfer.Sensors.HeatFlowSensor P_loss "Overall heat flow"
    annotation (Placement(transformation(extent={{50,-80},{70,-100}})));
  Modelica.Blocks.Continuous.Integrator E_loss_total "Integrate loss power"
    annotation (Placement(transformation(extent={{70,-60},{90,-40}})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor infinityHeatCapacitor(
    C=10000000000,
    T(fixed=true)) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={90,-90})));

  parameter VDCWorkbenchModels.Utilities.Types.StateOfCharge SOC_init=0.65
    "Initial value of SOC";

  replaceable VehicleComponents.Controllers.VDControl.BaseClasses.BaseVDC controller(
    cf=data.cf,
    cr=data.cr,
    steer_gain=data.steeringRatio,
    Torque_max_frontMotor=data.Torque_max_frontMotor,
    Torque_max_rearMotor=data.Torque_max_rearMotor,
    car_r=data.R0,
    m=data.m_vehicle,
    lf=data.wheelBase/2,
    lr=data.wheelBase/2,
    track_width=data.trackWidth,
    J=data.Jz_vehicle) annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  VehicleComponents.Powertrain.FrontMiddleAndRearIWM drivetrain(useHeatPort=true)
    annotation (Placement(transformation(extent={{-10,-40},{10,-20}})));
  VehicleComponents.Powertrain.BatteryAndRex powerSource(
    useHeatPort=true,
    SOC_init=SOC_init,
    includeGround=true) annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));
  VehicleComponents.Controllers.EnergyManagementAlgorithm.BEMA bEMA annotation (Placement(transformation(extent={{0,40},{20,60}})));
  Modelica.Blocks.Math.Gain maxH2_Idem(k=40)
    "Maximum current demand to the H2 cell"
    annotation (Placement(transformation(extent={{-66,-66},{-54,-54}})));
protected
  Utilities.Interfaces.EMAControls EMAcontrols
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
equation
  connect(P_loss.port_b, infinityHeatCapacitor.port) annotation (Line(points={{70,-90},{80,-90}}, color={191,0,0}));
  connect(E_loss_total.u,P_loss. Q_flow) annotation (Line(points={{68,-50},{60,-50},{60,-79}},
        color={0,0,127}));
  connect(E_loss_total.y, Eloss)
    annotation (Line(points={{91,-50},{110,-50}},
          color={0,0,127}));
  connect(P_loss.Q_flow, Ploss) annotation (Line(points={{60,-79},{60,-70},{110,-70}}, color={0,0,127}));
  connect(controller.controlBus, controlBus) annotation (Line(
      points={{-30,40},{-30,30},{30,30},{30,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(drivetrain.controlBus, controlBus) annotation (Line(
        points={{10,-26},{30,-26},{30,0},{100,0}},
        color={255,204,51},
        thickness=0.5));
  connect(powerSource.controlBus, controlBus) annotation (Line(
      points={{-40,-52},{-50,-52},{-50,30},{30,30},{30,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(drivetrain.flangeDriveFront, vehicle.flangeDriveFront) annotation (Line(points={{-10,-26},{-16,-26},{-16,0},{-10,0}},
        color={0,0,0}));
  connect(drivetrain.flangeWheelRL, vehicle.flangeWheelRL) annotation (Line(points={{-10,-34},{-14,-34},{-14,-6},{-10,-6}},
        color={0,0,0}));
  connect(drivetrain.flangeWheelRR, vehicle.flangeWheelRR) annotation (Line(points={{10,-34},{14,-34},{14,-6},{10,-6}}, color={0,0,0}));
  connect(drivetrain.heatPort, P_loss.port_a) annotation (Line(points={{-10,-40},{-10,-90},{50,-90}},
        color={191,0,0}));
  connect(powerSource.heatPort, P_loss.port_a) annotation (Line(points={{-40,-70},{-40,-80},{-10,-80},{-10,-90},{50,-90}},
        color={191,0,0}));
  connect(powerSource.pin_p, drivetrain.pin_p) annotation (Line(points={{-20,-54},{0,-54},{0,-40}}, color={0,0,255}));
  connect(powerSource.pin_n, drivetrain.pin_n) annotation (Line(points={{-20,-66},{6,-66},{6,-40}}, color={0,0,255}));
  connect(vehicle.heatPort, P_loss.port_a) annotation (Line(points={{-10,-10},{-20,-10},{-20,-44},{-10,-44},{-10,-90},{50,-90}},
        color={191,0,0}));
  connect(bEMA.controlBus, controlBus) annotation (Line(
      points={{10,40},{10,30},{30,30},{30,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(EMAcontrols, controlBus.eMAControls) annotation (Line(
      points={{-70,30},{30,30},{30,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(controller.v_scl, EMAcontrols.alpha_v)
    annotation (
      Line(points={{-42,50},{-72,50},{-72,30},{-70,30}},color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(controller.FrontRear_ratio, EMAcontrols.alpha_AD)
    annotation (
      Line(points={{-42,46},{-70,46},{-70,30}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(controller.TV_ratio, EMAcontrols.alpha_TV) annotation (
      Line(points={{-42,42},{-68,42},{-68,30},{-70,30}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(maxH2_Idem.y, powerSource.I_dem)
    annotation (Line(points={{-53.4,-60},{-42,-60}}, color={0,0,127}));
  connect(maxH2_Idem.u, EMAcontrols.alpha_FC) annotation (
      Line(points={{-67.2,-60},{-70,-60},{-70,30}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  annotation (
    Icon(graphics={
        Polygon(
          points={{56,60},{8,70},{-32,54},{-42,26},{-74,4},{-78,-4},{-78,-24},{-30,-34},{84,24},{84,42},{82,48},{56,60}},
          lineColor={135,135,135},
          smooth=Smooth.None,
          fillColor={170,213,255},
          fillPattern=FillPattern.Solid),
        Line(
          points={{56,60},{16,44},{6,16},{-26,-6},{-30,-14},{-30,-34}},
          color={135,135,135},
          smooth=Smooth.None),
        Line(
          points={{16,44},{-32,54}},
          color={135,135,135},
          smooth=Smooth.None),
        Line(
          points={{6,16},{-42,26}},
          color={135,135,135},
          smooth=Smooth.None),
        Line(
          points={{-26,-6},{-74,4}},
          color={135,135,135},
          smooth=Smooth.None),
        Ellipse(
          extent={{-18,-10},{-2,-44}},
          lineColor={135,135,135},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{56,28},{72,-6}},
          lineColor={135,135,135},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<p>
Vehicle&apos;s architecture intended for the IEEE VTS Motor Vehicles Challenge 2023,
see [<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2022</a>].
It considers
</p>
<ul>
  <li>
    a&nbsp;planar <code>vehicle</code> dynamics of the ROboMObil vehicle,
  </li>
  <li>
    a&nbsp;hybrid energy storage system <code>powerSource</code> consisting from fuel cell,
    hydrogen tank and battery,
  </li>
  <li>
    actuators:
    <ul>
      <li>
        <code>drivetrain</code>, which are two in-wheel electric motors installed in the rear
        axle and one central front motor, and
      </li>
      <li>
        a&nbsp;front steer-by-wire actuation, which is a&nbsp;component of the front axle of
        the planar <code>vehicle</code>.
      </li>
    </ul>
  </li>
</ul>
<p>
The actuators are manipulated by the vehicle&apos;s motion <code>controller</code> in order to track
a&nbsp;pre-defined reference velocity and track curvature.
</p>
<p>
The baseline energy management algorithm, called <code>bEMA</code>, determines the operating conditions for the two
energy storage devices (the hydrogen tank and the battery) and the three electric machines.
</p>
</html>"));
end VDCWorkbench2025;
