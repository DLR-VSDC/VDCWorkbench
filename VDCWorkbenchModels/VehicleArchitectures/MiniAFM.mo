within VDCWorkbenchModels.VehicleArchitectures;
partial model MiniAFM "Vehicle architecture of the DLR's MiniAFM"
  extends VehicleArchitectures.BaseArchitecture(
    v_Start=1,
    vehicle(
      useHeatPort=true,
      axleRear(
        wheelRight(phi_roll(fixed=true, start=0), w_roll(fixed=true))),
      axleFront(
        wheelRight(phi_roll(fixed=true, start=0), w_roll(fixed=true)))),
    redeclare Data.MiniAFMChassis data);
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

  replaceable VehicleComponents.Controllers.VDControl.BaseClasses.BaseSubBusses controller
    constrainedby VehicleComponents.Controllers.VDControl.BaseClasses.BaseSubBusses(
      filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat"),
      pathName="path",
      cf=data.cf,
      cr=data.cr,
      car_r=data.R0,
      m=data.m_vehicle,
      lf=data.wheelBase/2,
      lr=data.wheelBase/2,
      track_width=data.trackWidth,
      J=data.Jz_vehicle) annotation (Placement(transformation(extent={{20,40},{40,60}})));
  Data.MiniAFMPowertrain dataPowertrain annotation (Placement(transformation(extent={{0,80},{20,100}})));
  VehicleComponents.Powertrain.DrivetrainDifferentialIdeal powertrain(
    ratioEngineGear=dataPowertrain.ratioEngineGear,
    ratioFrontGear=dataPowertrain.ratioFrontGear,
    ratioRearGear=dataPowertrain.ratioRearGear) annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
equation
  connect(P_loss.port_b, infinityHeatCapacitor.port) annotation (Line(points={{70,-90},{80,-90}}, color={191,0,0}));
  connect(E_loss_total.u,P_loss. Q_flow) annotation (Line(points={{68,-50},{60,-50},{60,-79}},
        color={0,0,127}));
  connect(E_loss_total.y, Eloss)
    annotation (Line(points={{91,-50},{110,-50}},
          color={0,0,127}));
  connect(P_loss.Q_flow, Ploss) annotation (Line(points={{60,-79},{60,-70},{110,-70}}, color={0,0,127}));
  connect(controller.controlBus, controlBus) annotation (Line(
      points={{30,40},{30,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(vehicle.heatPort, P_loss.port_a) annotation (Line(points={{-10,-10},{-10,-90},{50,-90}},
        color={191,0,0}));
  connect(powertrain.flangeDriveFront, vehicle.flangeDriveFront) annotation (Line(points={{10,-26},{-16,-26},{-16,0},{-10,0}},color={0,0,0}));
  connect(powertrain.flangeDriveRear, vehicle.flangeDriveRear) annotation (Line(points={{10,-34},{0,-34},{0,-10}},color={0,0,0}));
  connect(powertrain.controlBus, controlBus) annotation (Line(
      points={{30,-30},{40,-30},{40,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(powertrain.heatPort, P_loss.port_a) annotation (Line(points={{10,-40},{-10,-40},{-10,-90},{50,-90}},color={191,0,0}));
  annotation (
    Icon(graphics={
        Polygon(
          points={{56,60},{8,70},{-32,54},{-42,26},{-74,4},{-78,-4},{-78,-24},{-30,-34},{84,24},{84,42},{82,48},{56,60}},
          lineColor={135,135,135},
          smooth=Smooth.None,
          fillColor={0,214,0},
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
    Documentation(
      info="<html>
<p>
Vehicle&apos;s architecture as published in the IEEE Open Journal of Vehicular Technology,
see [<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2026</a>].
It considers
</p>
<ul>
  <li>
    a&nbsp;planar <code>vehicle</code> dynamics of the <strong>MiniAFM</strong> vehicle,
  </li>
  <li>
    a&nbsp;powertrain actuator <code>powertrain</code> consisting from driving torque source
    and an ideal central differential splitting the torque between the front and the rear axle,
  </li>
  <li>
    a&nbsp;front steer-by-wire actuation, which is a&nbsp;component of the front axle of
    the planar <code>vehicle</code>.
  </li>
</ul>
<p>
The actuators are manipulated by the vehicle&apos;s motion <code>controller</code> in order to track
a&nbsp;pre-defined reference velocity and track.
</p>
</html>"));
end MiniAFM;
