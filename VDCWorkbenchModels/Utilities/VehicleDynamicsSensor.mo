within VDCWorkbenchModels.Utilities;
model VehicleDynamicsSensor
  "Ideal sensor to measure all vehicle dynamics quantities in the correct frame"
  extends Modelica.Icons.RoundSensor;

  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={100,0}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={100,0})));
  PlanarMechanics.Sensors.AbsolutePosition sensor_r_0(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Vehicle position resolved in world frame" annotation (Placement(transformation(extent={{-50,50},{-30,70}})));
  PlanarMechanics.Sensors.AbsoluteVelocity sensor_v_0(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Vehicle velocity resolved in world frame" annotation (Placement(transformation(extent={{-50,20},{-30,40}})));
  PlanarMechanics.Sensors.AbsoluteAcceleration sensor_a_0(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.world) "Vehicle acceleration resolved in world frame" annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  PlanarMechanics.Sensors.AbsoluteVelocity sensor_v_chassis(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) "Vehicle velocity w.r.t. vehicle frame" annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
  PlanarMechanics.Sensors.AbsoluteAcceleration sensor_a_chassis(resolveInFrame=Modelica.Mechanics.MultiBody.Types.ResolveInFrameA.frame_a) "Vehicle acceleration resolved in world frame" annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
  Modelica.Blocks.Math.Atan2 calc_beta
    annotation (Placement(transformation(extent={{12,-20},{32,-40}})));
  PlanarMechanics.Interfaces.Frame_a frame_CoG
    "Connect to the vehicles center of gravity"
    annotation (Placement(transformation(extent={{-116,-16},{-84,16}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{50,-10},{70,10}}),iconTransformation(extent={{50,-10},{70,10}})));
equation
  connect(sensor_r_0.frame_a, frame_CoG) annotation (Line(
      points={{-50,60},{-70,60},{-70,0},{-100,0}},
      color={95,95,95},
      thickness=0.5));
  connect(sensor_v_0.frame_a, frame_CoG) annotation (Line(
      points={{-50,30},{-70,30},{-70,0},{-100,0}},
      color={95,95,95},
      thickness=0.5));
  connect(sensor_a_0.frame_a, frame_CoG) annotation (Line(
      points={{-50,0},{-100,0}},
      color={95,95,95},
      thickness=0.5));
  connect(sensor_v_chassis.frame_a, frame_CoG) annotation (Line(
      points={{-50,-30},{-70,-30},{-70,0},{-100,0}},
      color={95,95,95},
      thickness=0.5));
  connect(sensor_a_chassis.frame_a, frame_CoG) annotation (Line(
      points={{-50,-60},{-70,-60},{-70,0},{-100,0}},
      color={95,95,95},
      thickness=0.5));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
        points={{60,0},{82,0},{82,-0.1},{100.1,-0.1}},
        color={255,204,51},
        thickness=0.5));
  connect(sensor_r_0.r[1], chassisBus.position_x) annotation (Line(points={{-29,65.75},{-28,65.75},{-28,64},{64,64},{64,2},{60,2},{60,0}}, color={0,0,127}));
  connect(sensor_r_0.r[2], chassisBus.position_y) annotation (Line(points={{-29,66.25},{66,66.25},{66,2},{60,2},{60,0}}, color={0,0,127}));
  connect(sensor_r_0.phi, chassisBus.position_phi) annotation (Line(points={{-29,54},{60,54},{60,0}}, color={0,0,127}));
  connect(sensor_v_0.v[1], chassisBus.velocity_dx) annotation (Line(points={{-29,35.75},{58,35.75},{58,4},{60,4},{60,0}}, color={0,0,127}));
  connect(sensor_v_0.v[2], chassisBus.velocity_dy) annotation (Line(points={{-29,36.25},{-28,36.25},{-28,34},{56,34},{56,2},{60,2},{60,0}}, color={0,0,127}));
  connect(sensor_v_0.w, chassisBus.velocity_dphi) annotation (Line(points={{-29,24},{52,24},{52,2},{60,2},{60,0}}, color={0,0,127}));
  connect(sensor_a_0.a[1], chassisBus.acceleration_ddx) annotation (Line(points={{-29,5.75},{-26,5.75},{-26,4},{48,4},{48,0},{60,0}}, color={0,0,127}));
  connect(sensor_a_0.a[2], chassisBus.acceleration_ddy) annotation (Line(points={{-29,6.25},{50,6.25},{50,0},{60,0}}, color={0,0,127}));
  connect(sensor_a_0.z, chassisBus.acceleration_ddphi) annotation (Line(points={{-29,-6},{54,-6},{54,0},{60,0}}, color={0,0,127}));
  connect(sensor_r_0.phi, chassisBus.yawAngle) annotation (Line(points={{-29,54},{-10,54},{-10,56},{62,56},{62,2},{60,2},{60,0}}, color={0,0,127}), Text(
      string="%second",
      index=3,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensor_v_0.w, chassisBus.yawRate) annotation (Line(points={{-29,24},{-24,24},{-24,26},{54,26},{54,2},{60,2},{60,0}}, color={0,0,127}), Text(
      string="%second",
      index=3,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensor_v_chassis.v[1], chassisBus.longitudinalVelocity) annotation (Line(points={{-29,-24.25},{0,-24.25},{0,-14},{58,-14},{58,0},{60,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(sensor_v_chassis.v[1], calc_beta.u2) annotation (Line(points={{-29,-24.25},{10,-24.25},{10,-24}}, color={0,0,127}));
  connect(sensor_v_chassis.w, calc_beta.u1) annotation (Line(points={{-29,-36},{10,-36}}, color={0,0,127}));
  connect(sensor_a_chassis.a[1], chassisBus.longitudinalAcceleration) annotation (Line(points={{-29,-54.25},{-28,-54.25},{-28,-54},{62,-54},{62,0},{60,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(sensor_a_chassis.a[2], chassisBus.lateralAcceleration) annotation (Line(points={{-29,-53.75},{-26,-53.75},{-26,-56},{64,-56},{64,0},{60,0}}, color={0,0,127}), Text(
      string="%second",
      index=3,
      extent={{6,-3},{6,-3}},
      horizontalAlignment=TextAlignment.Left));
  connect(calc_beta.y, chassisBus.sideSlipAngle) annotation (Line(points={{33,-30},{60,-30},{60,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));

  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Line(
          points={{70,0},{98,0}},
          color={255,204,51},
          thickness=0.5),
        Line(
          points={{-100,0},{-70,0}},
          color={95,95,95},
          thickness=0.5),
        Text(
          extent={{-150,120},{150,80}},
          textColor={0,0,255},
          textString="%name")}));
end VehicleDynamicsSensor;
