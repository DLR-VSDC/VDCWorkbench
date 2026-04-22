within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.GeoPFC;
model ControlAllocationTD "Time-discrete control allocation"
  //Implementation based on https://elib.dlr.de/105041/1/IV2016_PeterRitzer.pdf based on Python Code from Kenan Ahmic c DLR 2025

  parameter Modelica.Units.SI.Length lf = 1.2 "Distance of CoG to front axle";
  parameter Modelica.Units.SI.Length lr = 1.2 "Distance of CoG to rear axle";
  parameter Modelica.Units.SI.Torque maxTau=4*160 "Maximal torque";
  parameter Real Kspeedctrl = 4000 "P gain of velocity controller";
  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

protected
  parameter Modelica.Units.SI.Torque torque_max = maxTau;
  parameter Modelica.Units.SI.Torque torque_min = -maxTau;
  //Real vx_sens[2], vy_sens[2];
  Real vx_sens, vy_sens;

public
  Modelica.Blocks.Interfaces.RealOutput delta "Front steering angle [rad]"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Interfaces.RealOutput torque "Summarized propulsion torque [Nm]"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus annotation (Placement(
        transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={100,0})));
protected
  Modelica.Blocks.Interfaces.RealOutput kappaICR_geo
    "Desired curvature at instantious center of rotation of PFC"
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-10,-30}),iconTransformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={-10,-30})));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(
        transformation(extent={{50,-10},{70,10}})));
  VDCWorkbenchModels.Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
  Modelica.Blocks.Interfaces.RealOutput vveh "Absolute vehicle speed" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={-10,0})));
  Modelica.Blocks.Interfaces.RealOutput v_path
    "Velocity of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-10,-70})));
  Modelica.Blocks.Interfaces.RealOutput beta_geo
    "Desired side slip angle of geo pfc" annotation (Placement(transformation(
          extent={{0,-60},{-20,-40}}), iconTransformation(extent={{-52,62},{-30,
        84}})));

algorithm
  when sample(0, Ts) then
    // Geometric calculation of steering angle
    vx_sens :=cos(beta_geo);
    vy_sens :=sin(beta_geo) + kappaICR_geo*lf;

    delta :=atan2(vy_sens, vx_sens);

    // Total motortorque speed controller
    torque :=min(1.0*torque_max, max(torque_min, Kspeedctrl*(v_path - vveh)));

  end when;

equation

  connect(chassisBus,controlBus. chassisBus) annotation (Line(
      points={{60,0},{98,0},{98,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5,
      smooth=Smooth.None));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
      points={{80,-40},{80,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(v_path, motionDemandBus.v_path)
    annotation (Line(points={{-10,-70},{22,-70},{22,-42},{80,-42},{80,-40}},
        color={0,0,127}));
  connect(beta_geo, motionDemandBus.beta_geo) annotation (Line(points={{-10,-50},{20,-50},{20,-40},{80,-40}},
        color={0,0,127}));
  connect(kappaICR_geo, motionDemandBus.kappaICR_geo) annotation (Line(points={{-10,-30},{20,-30},{20,-38},{80,-38},{80,-40}},
        color={0,0,127}));
  connect(vveh, chassisBus.longitudinalVelocity) annotation (Line(points={{-10,0},{60,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Polygon(
          points={{-100,-86},{-76,100},{80,100},{100,-86},{-100,-86}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Forward),
        Text(
          extent={{-100,60},{100,0}},
          textColor={255,255,255},
          textString="Control
Allocation"),
        Line(
          points={{-80,-80},{-80,-40},{-30,-40},{-30,-20},{20,-20},{20,-60},{70,-60},{70,-10}},
          color={255,255,255},
          pattern=LinePattern.Dot),
        Ellipse(
          extent={{-86,-34},{-74,-46}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,-14},{-24,-26}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-54},{26,-66}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end ControlAllocationTD;
