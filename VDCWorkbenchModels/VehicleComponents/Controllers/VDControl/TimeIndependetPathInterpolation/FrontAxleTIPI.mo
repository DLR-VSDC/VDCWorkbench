within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.TimeIndependetPathInterpolation;
model FrontAxleTIPI "Front axle time-independent path interpolation"
  parameter Real e_long_gain=80 "TIPI Controller gain to force e_long to 0";
  parameter Modelica.Units.SI.Position s_start=0 "Arc length value at start position";
  parameter String filePath = ModelicaServices.ExternalReferences.loadResource(
    "modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat")
    "File where path table pathName is stored" annotation (
      Dialog(
        group="Path data",
        loadSelector(
          filter="Matlab files(*.mat)",
          caption="Open data file")));
  parameter String pathName = "path" "Table name in filePath" annotation (Dialog(group="Path data"));
  parameter Modelica.Units.SI.Position maxArcLength = 22.737000000000002 "Maximum arc length value on path file" annotation (Dialog(group="Path data"));

  parameter Modelica.Units.SI.Length lf =0.1805 "Distance of CoG to front axle";

  Real sDot "Time derivative of arc length";
  Real tvI_P[2] "Tangent of desired path in inertial frame I (normalized)";
  Real nvI_P[2]
    "Vector normal to tvI_P; rotated tvI_P +90° in inertial frame I (normalized)";
  //Real vI_C[2] "Vehicle speed in inertial frame of reference"; // -> Not considered since this value is a direct input
  Real e[2] "Distance vector from path reference to vehicle position in inertial frame I";
  Real e_lat "Distance of vehicle to path in direction of nvI_P";
  Real e_long "Distance of vehicle to path in direction of nvI_P";
  Real kappa
    "Curvature of path, derivative of psi_ref with respect to parameter s";
  Real lambda[5];
  Real sI_front[2] "Front axle position in inertial frame";

  Modelica.Blocks.Tables.CombiTable1Ds combiTablePath(
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
    columns={2,3,4,5,6},
    tableOnFile=true,
    fileName=filePath,
    tableName=pathName) annotation (Placement(transformation(extent={{40,30},{60,50}})));

protected
  Modelica.Blocks.Interfaces.RealOutput sI_C[3] "Vehicle Position in inertial frame of reference"
    annotation (Placement(
      transformation(extent={{0,-70},{-20,-50}})));
  Modelica.Blocks.Interfaces.RealOutput vI_C[2] "Vehicle speed in inertial frame I"
    annotation (Placement(
      transformation(extent={{0,-100},{-20,-80}})));
public
  Modelica.Blocks.Sources.RealExpression realExpression_sDot(y=sDot)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Modelica.Blocks.Routing.RealPassThrough sampler
    annotation (Placement(transformation(extent={{0,30},{20,50}})));
  Modelica.Blocks.Continuous.Integrator sIntegrator(y_start=s_start)
    annotation (Placement(transformation(extent={{-60,30},{-40,50}})));
  Modelica.Blocks.Sources.RealExpression realExpression_e_long(y=e_long)
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Modelica.Blocks.Sources.RealExpression realExpression_e_lat(y=e_lat)
    annotation (Placement(transformation(extent={{40,-62},{60,-42}})));
  Utilities.Blocks.Modulo modulo(k=maxArcLength) annotation (Placement(transformation(extent={{-30,30},{-10,50}})));
  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={100,0})));
  Modelica.Blocks.Interfaces.RealInput v_scl "Down scale vector for velocity "
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
protected
  VDCWorkbenchModels.Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{10,-90},{30,-70}})));
initial equation
//kappa = Bahn.y[5];
equation
  tvI_P = {cos(lambda[3]),sin(lambda[3])};
  nvI_P = [0,-1; 1,0]*tvI_P;  //Rotate tvI_P around 90°
  //vI_C = [cos(sI_C[3]),-sin(sI_C[3]); sin(sI_C[3]),cos(sI_C[3])]*vC_C; // -> Not considered since this value is a direct input

  // calculate center of front axle
  sI_front[1] = sI_C[1] + lf * cos(sI_C[3]);
  sI_front[2] = sI_C[2] + lf * sin(sI_C[3]);

  e = sI_front - lambda[1:2]; // Calculate distance error in inertial frame I
  e_long = e*tvI_P;  // Calculate distance error in direction of path tangent
  e_lat = e*nvI_P;  // Calculate distance error in direction normal to path tangent

  // Calculate reference velocity
  //sDot=(cos(lambda[3]-sI_C[3])*vC_C[1]+sin(lambda[3]-sI_C[3])*vC_C[2])/(1-e_lat*kappa)+e_long_gain*e_long;
  sDot=(cos(lambda[3])*vI_C[1]+sin(lambda[3])*vI_C[2])/(1-e_lat*kappa)+e_long_gain*e_long;
  //lambda[:] = combiTablePath.y[:];
  lambda[1] = combiTablePath.y[1] "x-position";
  lambda[2] = combiTablePath.y[2] "y-position";
  lambda[3] = combiTablePath.y[3] "psi orientation";
  lambda[4] = combiTablePath.y[4]*v_scl "scaled long speed";
  lambda[5] = combiTablePath.y[5] "curvature";
  kappa = combiTablePath.y[5];
  //if sIntegrator.y > (maxArcLength-1.0) then
  //  terminate("Vehicle reached end of path");
  //end if;

  connect(combiTablePath.u, sampler.y) annotation (Line(
      points={{38,40},{21,40}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(modulo.y, sampler.u) annotation (Line(points={{-9,40},{-2,40}},
        color={0,0,127}));
  connect(realExpression_sDot.y, sIntegrator.u) annotation (Line(points={{-79,40},{-62,40}},
        color={0,0,127}));
  connect(sIntegrator.y, modulo.u) annotation (Line(points={{-39,40},{-32,40}},
        color={0,0,127}));
  connect(realExpression_sDot.y, motionDemandBus.s_dot) annotation (Line(points={{-79,40},{-70,40},{-70,-2},{80,-2},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(sampler.y, motionDemandBus.arc_length) annotation (Line(points={{21,40},{30,40},{30,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(realExpression_e_long.y, motionDemandBus.e_long) annotation (Line(
        points={{61,-30},{80,-30},{80,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(realExpression_e_lat.y, motionDemandBus.e_lat) annotation (Line(
        points={{61,-52},{82,-52},{82,0},{80,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
        points={{80,0},{80,-0.1},{100.1,-0.1}},
        color={255,204,51},
        thickness=0.5));
  connect(combiTablePath.y[1], motionDemandBus.x_path) annotation (Line(points={{61,40},{64,40},{64,42},{82,42},{82,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,2},{6,2}},
        horizontalAlignment=TextAlignment.Left));
  connect(combiTablePath.y[2], motionDemandBus.y_path) annotation (Line(points={{61,40},{80,40},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(combiTablePath.y[3], motionDemandBus.psi_path) annotation (Line(points={{61,40},{64,40},{64,34},{74,34},{74,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,2},{6,2}},
        horizontalAlignment=TextAlignment.Left));
  connect(combiTablePath.y[4], motionDemandBus.v_path) annotation (Line(points={{61,40},{66,40},{66,36},{76,36},{76,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,2},{6,2}},
        horizontalAlignment=TextAlignment.Left));
  connect(combiTablePath.y[5], motionDemandBus.kappa_path) annotation (Line(points={{61,40},{68,40},{68,38},{78,38},{78,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=3,
        extent={{6,2},{6,2}},
        horizontalAlignment=TextAlignment.Left));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{20,-80},{100.1,-80},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(sI_C[1], chassisBus.position_x) annotation (Line(points={{-10,-63.3333},{18,-63.3333},{18,-80},{20,-80}},
        color={0,0,127}));
  connect(sI_C[2], chassisBus.position_y) annotation (Line(points={{-10,-60},{20,-60},{20,-80}},
        color={0,0,127}));
  connect(sI_C[3], chassisBus.yawAngle) annotation (Line(points={{-10,-56.6667},{22,-56.6667},{22,-80},{20,-80}},
        color={0,0,127}));
  connect(vI_C[1], chassisBus.velocity_dx) annotation (Line(points={{-10,-92.5},{20,-92.5},{20,-80}}, color={0,0,127}));
  connect(vI_C[2], chassisBus.velocity_dy) annotation (Line(points={{-10,-87.5},{18,-87.5},{18,-80},{20,-80}}, color={0,0,127}));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(
          points={{0,100},{100,-100},{-100,-100},{0,100}},
          lineColor={28,108,200},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{-92,-86},{-68,-20},{-24,30},{64,98}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-74,-96},{-46,-28},{-2,16},{96,90}},
          color={28,108,200},
          pattern=LinePattern.Dash,
          thickness=0.5),
        Line(
          points={{-38,-98},{0,-28},{98,54}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-14,30},{-14,4},{10,4}},
          color={238,46,47},
          thickness=0.5),
        Text(
          extent={{-28,2},{0,-12}},
          textColor={238,46,47},
          textString="s"),
        Text(
          extent={{-30,-70},{90,-100}},
          textColor={0,0,0},
          textString="FA-TIPI")}),
    Diagram(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Text(
          extent={{10,100},{90,50}},
          textColor={28,108,200},
          textString="Input arc length
'X-Position        '
'Y-Position        '
'Psi-Orientation   '
'longitudinal Speed'
'Curvature         '
")}));
end FrontAxleTIPI;
