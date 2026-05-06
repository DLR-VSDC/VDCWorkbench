within VDCWorkbenchModels.Utilities.Blocks;
model MapFmuOutputsBusSignals "Map control bus signals into FMU's output bus"
  extends Modelica.Blocks.Icons.Block;
  import Modelica.Units.Conversions.from_deg;

  parameter Real steeringRatio = 16;

  Interfaces.ControlBus controlBus annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-100,0})));
  Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(
        transformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={200,0}),
        iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={100,0})));

protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{-50,80},{-30,100}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{-30,-140},{-10,-120}})));
  Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-70,-30},{-50,-10}})));

  // Vehicle signals
  Modelica.Blocks.Interfaces.RealOutput vehicle_Xposition
    "x coordinate of the CoG of the vhicle in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,190},{120,210}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_Yposition
    "y coordinate of the CoG of the vhicle in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,170},{120,190}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_sideSlipAngle(
    min=-from_deg(10),
    max=from_deg(10),
    nominal=from_deg(1)) "Vehicle side slip angle #RLobs"
    annotation (Placement(transformation(extent={{100,150},{120,170}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_yawRate(
    min=-1.0,
    max=1.0,
    nominal=0.15) "Yaw rate of the vehicle #RLobs" annotation (Placement(
        transformation(extent={{100,130},{120,150}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_absoluteVelocity(
    min=-50,
    max=50,
    nominal=20) "Absolute velocity of the vehicle #RLobs"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_lateralAcceleration(
    min=-10,
    max=10,
    nominal=3) "Vehicle lateral acceleration #RLobs"
    annotation (Placement(transformation(extent={{100,110},{120,130}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_longitudinalAcceleration(
    min=-4.0,
    max=4.0,
    nominal=1.5) "Vehicle longitudinal acceleration #RLobs"
    annotation (Placement(transformation(extent={{100,90},{120,110}})));

  // Errors
  Modelica.Blocks.Interfaces.RealOutput lateralDisplacement_error(
    unit="m",
    min=-2.5,
    max=2.5,
    nominal=0.5)
    "Lateral displacement of the vehicle CoG with respect to the path reference point. Positive value indicates that the vehicle needs to steer to the right (i.e. negative steeringWheelAngle) to reach the path. #RLobs"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.RealOutput longitudinalVelocity_error(
    min=-10.0,
    max=10.0,
    nominal=3.0)
    "Reference velocity - absolute vehicle velocity in path direction #RLobs"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput orientation_error(
    unit="rad",
    min=-from_deg(45),
    max=from_deg(45),
    nominal=from_deg(5))
    "Heading angle of the vehicle - tangential orientation of the path #RLobs"
    annotation (Placement(transformation(extent={{100,10},{120,30}})));

     // Path signals
  Modelica.Blocks.Interfaces.RealOutput path_curvature(
    unit="m-1",
    min=-0.1,
    max=0.1,
    nominal=0.02) "Curvature of the path #RLobs"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput path_Yposition
    "y coordinate of the current path referencepoint in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
  Modelica.Blocks.Interfaces.RealOutput path_Xposition
    "x coordinate of the current path referencepoint in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,-30},{120,-10}})));
  Modelica.Blocks.Interfaces.RealOutput path_velocity(
    min=-50.0,
    max=50.0,
    nominal=20) "Combined commanded torque (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Interfaces.RealOutput arcLength(unit="m")
    "arc length of the reference point on the path #RL"
    annotation (Placement(transformation(extent={{100,-90},{120,-70}})));

    // Controller related signals
  Modelica.Blocks.Interfaces.RealOutput baselineController_steeringWheelAngle(
    unit="rad",
    min=-from_deg(400),
    max=from_deg(400),
    nominal=from_deg(40))
    "Steering wheel angle as commanded by the baseline controller #RLobs"
    annotation (Placement(transformation(extent={{100,-110},{120,-90}})));
  Modelica.Blocks.Interfaces.RealOutput combined_steeringWheelAngle(
    min=-from_deg(400),
    max=from_deg(400),
    nominal=from_deg(40))
    "Combined commanded steering wheel angle (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-130},{120,-110}})));
  Modelica.Blocks.Interfaces.RealOutput baselineController_torque(
    min=-640,
    max=640,
    nominal=400) "Torque commanded by the baseline controller #RLobs"
    annotation (Placement(transformation(extent={{100,-150},{120,-130}})));
  Modelica.Blocks.Interfaces.RealOutput combined_torque(
    min=-640,
    max=640,
    nominal=400) "Combined commanded torque (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-170},{120,-150}})));
  Modelica.Blocks.Interfaces.RealOutput residualRL_steeringWheelAngle(
    unit="rad",
    min=-from_deg(4)*steeringRatio,
    max=from_deg(4)*steeringRatio,
    nominal=from_deg(4)*steeringRatio) "Steering wheel angle commanded by the DRL agent #RLobs"
    annotation (Placement(transformation(extent={{100,-190},{120,-170}})));
  Modelica.Blocks.Interfaces.RealOutput residualRL_torque(
    unit="N.m",
    min=-150,
    max=150,
    nominal=150) "Torque commanded by the DRL agent #RLobs"
    annotation (Placement(transformation(extent={{100,-210},{120,-190}})));

  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_yawRate
    annotation (Placement(transformation(extent={{48,134},{60,146}})));
public
  Modelica.Blocks.Math.Add calc_orientation_error(k2=-1)
    annotation (Placement(transformation(extent={{-20,10},{0,30}})));
  VelocityError calcVelocityError annotation (Placement(transformation(extent={{-20,50},{0,70}})));
protected
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_Xposition
    annotation (Placement(transformation(extent={{48,194},{60,206}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_Yposition
    annotation (Placement(transformation(extent={{48,174},{60,186}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_sideSlipAngle
    annotation (Placement(transformation(extent={{48,154},{60,166}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_absoluteVelocity
    annotation (Placement(transformation(extent={{48,74},{60,86}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_lateralAcceleration
    annotation (Placement(transformation(extent={{48,114},{60,126}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_longitudinalAcceleration
    annotation (Placement(transformation(extent={{48,94},{60,106}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_lateralDisplacement_error
    annotation (Placement(transformation(extent={{48,34},{60,46}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_longitudinalVelocity_error
    annotation (Placement(transformation(extent={{48,54},{60,66}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_orientation_error
    annotation (Placement(transformation(extent={{48,14},{60,26}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_curvature
    annotation (Placement(transformation(extent={{48,-6},{60,6}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_Xposition
    annotation (Placement(transformation(extent={{48,-26},{60,-14}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_Yposition
    annotation (Placement(transformation(extent={{48,-46},{60,-34}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_velocity
    annotation (Placement(transformation(extent={{48,-66},{60,-54}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_arcLength
    annotation (Placement(transformation(extent={{48,-86},{60,-74}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_baselineController_steeringWheelAngle
    annotation (Placement(transformation(extent={{48,-106},{60,-94}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_combined_steeringWheelAngle
    annotation (Placement(transformation(extent={{48,-126},{60,-114}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_baselineController_torque
    annotation (Placement(transformation(extent={{48,-146},{60,-134}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_combined_torque
    annotation (Placement(transformation(extent={{48,-166},{60,-154}})));
public
  Modelica.Blocks.Interfaces.RealInput residualRL_steeringWheelAngle_in
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput residualRL_torque_in
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
equation
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{-20,-130},{-20,-40},{-70,-40},{-70,-0.1},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{-40,90},{-70,90},{-70,-0.1},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
      points={{-60,-20},{-70,-20},{-70,0},{-99.9,0},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(orientation_error, orientation_error)
    annotation (Line(points={{110,20},{110,20}}, color={0,0,127}));
  connect(calc_orientation_error.u1, chassisBus.yawAngle) annotation (Line(
        points={{-22,26},{-42,26},{-42,90},{-40,90}},             color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(calc_orientation_error.u2, motionDemandBus.psi_path) annotation (Line(
        points={{-22,14},{-40,14},{-40,-14},{-58,-14},{-58,-18},{-60,-18},{-60,-20}},
        color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.chassisBus, chassisBus) annotation (Line(
      points={{-20,64},{-40,64},{-40,90}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.motionDemandBus, motionDemandBus) annotation (Line(
      points={{-20,56},{-60,56},{-60,-20}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_yawRate.u, chassisBus.yawRate) annotation (
      Line(points={{46.8,140},{-18,140},{-18,94},{-40,94},{-40,90}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_torque.y, combined_torque)
    annotation (Line(points={{60.6,-160},{110,-160}}, color={0,0,127}));
  connect(realPassThrough_baselineController_torque.y,
    baselineController_torque)
    annotation (Line(points={{60.6,-140},{110,-140}}, color={0,0,127}));
  connect(realPassThrough_combined_steeringWheelAngle.y,
    combined_steeringWheelAngle)
    annotation (Line(points={{60.6,-120},{110,-120}}, color={0,0,127}));
  connect(realPassThrough_baselineController_steeringWheelAngle.y,
    baselineController_steeringWheelAngle)
    annotation (Line(points={{60.6,-100},{110,-100}}, color={0,0,127}));
  connect(realPassThrough_arcLength.y, arcLength)
    annotation (Line(points={{60.6,-80},{110,-80}},   color={0,0,127}));
  connect(realPassThrough_path_Yposition.y, path_Yposition)
    annotation (Line(points={{60.6,-40},{110,-40}}, color={0,0,127}));
  connect(realPassThrough_path_Xposition.y, path_Xposition)
    annotation (Line(points={{60.6,-20},{110,-20}}, color={0,0,127}));
  connect(realPassThrough_path_curvature.y, path_curvature)
    annotation (Line(points={{60.6,0},{110,0}},     color={0,0,127}));
  connect(realPassThrough_orientation_error.y, orientation_error)
    annotation (Line(points={{60.6,20},{110,20}},   color={0,0,127}));
  connect(realPassThrough_longitudinalVelocity_error.y,
    longitudinalVelocity_error)
    annotation (Line(points={{60.6,60},{110,60}},
        color={0,0,127}));
  connect(realPassThrough_lateralDisplacement_error.y,
    lateralDisplacement_error)
    annotation (Line(points={{60.6,40},{110,40}}, color={0,0,127}));
  connect(realPassThrough_vehicle_longitudinalAcceleration.y,
    vehicle_longitudinalAcceleration)
    annotation (Line(points={{60.6,100},{110,100}},
        color={0,0,127}));
  connect(realPassThrough_vehicle_lateralAcceleration.y,
    vehicle_lateralAcceleration)
    annotation (Line(points={{60.6,120},{110,120}},
        color={0,0,127}));
  connect(realPassThrough_vehicle_absoluteVelocity.y, vehicle_absoluteVelocity)
    annotation (Line(points={{60.6,80},{110,80}}, color={0,0,127}));
  connect(realPassThrough_vehicle_yawRate.y, vehicle_yawRate)
    annotation (Line(points={{60.6,140},{110,140}}, color={0,0,127}));
  connect(realPassThrough_vehicle_sideSlipAngle.y, vehicle_sideSlipAngle)
    annotation (Line(points={{60.6,160},{110,160}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Yposition.y, vehicle_Yposition)
    annotation (Line(points={{60.6,180},{110,180}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Xposition.y, vehicle_Xposition)
    annotation (Line(points={{60.6,200},{110,200}}, color={0,0,127}));
  connect(realPassThrough_path_velocity.y, path_velocity)
    annotation (Line(points={{60.6,-60},{110,-60}},   color={0,0,127}));
  connect(vehicle_Xposition, fmuOutputsBus.vehicle_Xposition) annotation (Line(
        points={{110,200},{210,200},{210,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(vehicle_Yposition, fmuOutputsBus.vehicle_Yposition) annotation (Line(
        points={{110,180},{208,180},{208,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(vehicle_sideSlipAngle, fmuOutputsBus.vehicle_sideSlipAngle)
    annotation (Line(points={{110,160},{206,160},{206,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(vehicle_yawRate, fmuOutputsBus.vehicle_yawRate) annotation (Line(
        points={{110,140},{200,140},{200,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-2,-2},{-2,-5}},
        horizontalAlignment=TextAlignment.Right));
  connect(vehicle_absoluteVelocity, fmuOutputsBus.vehicle_absoluteVelocity)
    annotation (Line(points={{110,80},{198,80},{198,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(vehicle_lateralAcceleration, fmuOutputsBus.vehicle_lateralAcceleration)
    annotation (Line(points={{110,120},{202,120},{202,0},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_longitudinalAcceleration, fmuOutputsBus.vehicle_longitudinalAcceleration)
    annotation (Line(points={{110,100},{200,100},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(lateralDisplacement_error, fmuOutputsBus.lateralDisplacement_error)
    annotation (Line(points={{110,40},{194,40},{194,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(longitudinalVelocity_error, fmuOutputsBus.longitudinalVelocity_error)
    annotation (Line(points={{110,60},{196,60},{196,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(orientation_error, fmuOutputsBus.orientation_error) annotation (Line(
        points={{110,20},{192,20},{192,0},{200,0}},               color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(path_curvature, fmuOutputsBus.path_curvature) annotation (Line(points={{110,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(path_Xposition, fmuOutputsBus.path_Xposition) annotation (Line(points={{110,-20},{192,-20},{192,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(path_Yposition, fmuOutputsBus.path_Yposition) annotation (Line(points={{110,-40},{194,-40},{194,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(path_velocity, fmuOutputsBus.path_velocity) annotation (Line(points={{110,-60},{196,-60},{196,-2},{198,-2},{198,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(arcLength, fmuOutputsBus.arcLength) annotation (Line(points={{110,-80},{198,-80},{198,0},{200,0}},
                                                             color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(baselineController_steeringWheelAngle, fmuOutputsBus.baselineController_steeringWheelAngle)
    annotation (Line(points={{110,-100},{200,-100},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(combined_steeringWheelAngle, fmuOutputsBus.combined_steeringWheelAngle)
    annotation (Line(points={{110,-120},{202,-120},{202,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(baselineController_torque, fmuOutputsBus.baselineController_torque)
    annotation (Line(points={{110,-140},{200,-140},{200,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(combined_torque, fmuOutputsBus.combined_torque) annotation (Line(
        points={{110,-160},{206,-160},{206,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(residualRL_steeringWheelAngle, fmuOutputsBus.residualRL_steeringWheelAngle)
    annotation (Line(points={{110,-180},{208,-180},{208,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(residualRL_torque, fmuOutputsBus.residualRL_torque) annotation (Line(
        points={{110,-200},{210,-200},{210,0},{200,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(calc_orientation_error.y, realPassThrough_orientation_error.u)
    annotation (Line(points={{1,20},{46.8,20}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Xposition.u, chassisBus.position_x)
    annotation (Line(points={{46.8,200},{-24,200},{-24,100},{-42,100},{-42,90},{-40,90}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_Yposition.u, chassisBus.position_y)
    annotation (Line(points={{46.8,180},{-22,180},{-22,98},{-40,98},{-40,90}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_sideSlipAngle.u, chassisBus.sideSlipAngle)
    annotation (Line(points={{46.8,160},{-20,160},{-20,96},{-40,96},{-40,90}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.vehicle_absolute_velocity, realPassThrough_vehicle_absoluteVelocity.u)
    annotation (Line(points={{1,66},{30,66},{30,80},{46.8,80}}, color={0,0,127}));
  connect(realPassThrough_vehicle_lateralAcceleration.u, chassisBus.lateralAcceleration)
    annotation (Line(points={{46.8,120},{10,120},{10,92},{-40,92},{-40,90}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_longitudinalAcceleration.u, chassisBus.longitudinalAcceleration)
    annotation (Line(points={{46.8,100},{12,100},{12,90},{-40,90}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_lateralDisplacement_error.u, motionDemandBus.e_lat)
    annotation (Line(points={{46.8,40},{18,40},{18,-16},{-56,-16},{-56,-20},{-60,-20}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_longitudinalVelocity_error.u, calcVelocityError.longitudinal_velocity_error)
    annotation (Line(points={{46.8,60},{1,60}},
        color={0,0,127}));
  connect(realPassThrough_path_curvature.u, motionDemandBus.kappa_path)
    annotation (Line(points={{46.8,0},{20,0},{20,-18},{-60,-18},{-60,-20}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_Xposition.u, motionDemandBus.x_path) annotation
    (Line(points={{46.8,-20},{-60,-20}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_Yposition.u, motionDemandBus.y_path) annotation
    (Line(points={{46.8,-40},{20,-40},{20,-22},{-60,-22},{-60,-20}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_velocity.u, motionDemandBus.v_path) annotation (
      Line(points={{46.8,-60},{18,-60},{18,-24},{-60,-24},{-60,-20}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_arcLength.u, motionDemandBus.arc_length) annotation (
      Line(
        points={{46.8,-80},{16,-80},{16,-26},{-60,-26},{-60,-20}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{-6,3},{-6,3}},
        horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_baselineController_steeringWheelAngle.u,
    chassisControlBus.baseline_steering) annotation (Line(points={{46.8,-100},{28,-100},{28,-128},{-20,-128},{-20,-130}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_steeringWheelAngle.u, chassisControlBus.steeringWheelAngle)
    annotation (Line(points={{46.8,-120},{30,-120},{30,-130},{-20,-130}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_baselineController_torque.u, chassisControlBus.baseline_torque)
    annotation (Line(points={{46.8,-140},{30,-140},{30,-132},{-20,-132},{-20,-130}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_torque.u, chassisControlBus.torque)
    annotation (Line(points={{46.8,-160},{28,-160},{28,-134},{-20,-134},{-20,-130}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(residualRL_steeringWheelAngle_in, residualRL_steeringWheelAngle) annotation (Line(points={{-120,-40},{-80,-40},{-80,-180},{110,-180}},
        color={0,0,127}));
  connect(residualRL_torque_in, residualRL_torque)
    annotation (Line(points={{-120,-80},{-90,-80},{-90,-200},{110,-200}},
        color={0,0,127}));
  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-52,86},{-48,-22}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Line(
          points={{-52,38},{-100,0}},
          color={255,204,51},
          thickness=0.5),
        Line(
          points={{100,0},{50,42}},
          color={255,204,51},
          thickness=0.5),
        Polygon(
          points={{-48,86},{-30,76},{-48,66},{-48,86}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-48,64},{-30,54},{-48,44},{-48,64}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-48,42},{-30,32},{-48,22},{-48,42}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-48,20},{-30,10},{-48,0},{-48,20}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-48,-2},{-30,-12},{-48,-22},{-48,-2}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{46,86},{50,-22}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Polygon(
          points={{28,86},{46,76},{28,66},{28,86}},
          lineColor={0,0,127},
          lineThickness=0.5),
        Polygon(
          points={{28,64},{46,54},{28,44},{28,64}},
          lineColor={0,0,127},
          lineThickness=0.5),
        Polygon(
          points={{28,42},{46,32},{28,22},{28,42}},
          lineColor={0,0,127},
          lineThickness=0.5),
        Polygon(
          points={{28,20},{46,10},{28,0},{28,20}},
          lineColor={0,0,127},
          lineThickness=0.5),
        Polygon(
          points={{28,-2},{46,-12},{28,-22},{28,-2}},
          lineColor={0,0,127},
          lineThickness=0.5),
        Line(
          points={{-38,76},{-18,76},{14,32},{28,32}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{-40,54},{-32,54},{-16,54},{6,76},{28,76}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{-40,-12},{28,-12}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{-100,-40},{-20,-40},{6,54},{28,54}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{-100,-82},{-14,-82},{14,10},{28,10}},
          color={0,0,127},
          thickness=0.5)}),
    Diagram(
      coordinateSystem(preserveAspectRatio=true,
      extent={{-100,-160},{200,160}})));
end MapFmuOutputsBusSignals;
