within VDCWorkbenchModels.Utilities.Blocks;
model MapFmuOutputsBusSignals "Map control bus signals into FMU's output bus"
  extends Modelica.Blocks.Icons.Block;
  import Modelica.Units.Conversions.from_deg;

  parameter Real steeringRatio = 16;

  Interfaces.ControlBus controlBus annotation (Placement(
        transformation(extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-100,0}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-100,0})));
protected
  // Vehicle signals
  Modelica.Blocks.Interfaces.RealOutput vehicle_Xposition
    "x coordinate of the CoG of the vhicle in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,150},{120,170}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_Yposition
    "y coordinate of the CoG of the vhicle in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,130},{120,150}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_sideSlipAngle(
    min=-from_deg(10),
    max=from_deg(10),
    nominal=from_deg(1)) "Vehicle side slip angle #RLobs"
    annotation (Placement(transformation(extent={{100,110},{120,130}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_yawRate(
    min=-1.0,
    max=1.0,
    nominal=0.15) "Yaw rate of the vehicle #RLobs" annotation (Placement(
        transformation(extent={{100,90},{120,110}}),  iconTransformation(extent
          ={{100,126},{112,138}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_absoluteVelocity(
    min=-50,
    max=50,
    nominal=20) "Absolute velocity of the vehicle #RLobs"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_lateralAcceleration(
    min=-10,
    max=10,
    nominal=3) "Vehicle lateral acceleration #RLobs"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_longitudinalAcceleration(
    min=-4.0,
    max=4.0,
    nominal=1.5) "Vehicle longitudinal acceleration #RLobs"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));

  // Errors
  Modelica.Blocks.Interfaces.RealOutput lateralDisplacement_error(
    unit="m",
    min=-2.5,
    max=2.5,
    nominal=0.5)
    "Lateral displacement of the vehicle CoG with respect to the path reference point. Positive value indicates that the vehicle needs to steer to the right (i.e. negative steeringWheelAngle) to reach the path. #RLobs"
    annotation (Placement(transformation(extent={{100,10},{120,30}})));
  Modelica.Blocks.Interfaces.RealOutput longitudinalVelocity_error(
    min=-10.0,
    max=10.0,
    nominal=3.0)
    "Reference velocity - absolute vehicle velocity in path direction #RLobs"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput orientation_error(
    unit="rad",
    min=-from_deg(45),
    max=from_deg(45),
    nominal=from_deg(5))
    "Heading angle of the vehicle - tangential orientation of the path #RLobs"
    annotation (Placement(transformation(extent={{100,-30},{120,-10}})));

     // Path signals
  Modelica.Blocks.Interfaces.RealOutput path_curvature(
    unit="m-1",
    min=-0.1,
    max=0.1,
    nominal=0.02) "Curvature of the path #RLobs"
    annotation (Placement(transformation(extent={{100,-50},{120,-30}})));
  Modelica.Blocks.Interfaces.RealOutput path_Yposition
    "y coordinate of the current path referencepoint in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,-90},{120,-70}})));
  Modelica.Blocks.Interfaces.RealOutput path_Xposition
    "x coordinate of the current path referencepoint in the inertial frame #RL"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Interfaces.RealOutput path_velocity(
    min=-50.0,
    max=50.0,
    nominal=20) "Combined commanded torque (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-110},{120,-90}})));
  Modelica.Blocks.Interfaces.RealOutput arcLength(unit="m")
    "arc length of the reference point on the path #RL"
    annotation (Placement(transformation(extent={{100,-130},{120,-110}})));

    // Controller related signals
  Modelica.Blocks.Interfaces.RealOutput baselineController_steeringWheelAngle(
    unit="rad",
    min=-from_deg(400),
    max=from_deg(400),
    nominal=from_deg(40))
    "Steering wheel angle as commanded by the baseline controller #RLobs"
    annotation (Placement(transformation(extent={{100,-150},{120,-130}})));
  Modelica.Blocks.Interfaces.RealOutput combined_steeringWheelAngle(
    min=-from_deg(400),
    max=from_deg(400),
    nominal=from_deg(40))
    "Combined commanded steering wheel angle (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-170},{120,-150}})));
  Modelica.Blocks.Interfaces.RealOutput baselineController_torque(
    min=-640,
    max=640,
    nominal=400) "Torque commanded by the baseline controller #RLobs"
    annotation (Placement(transformation(extent={{100,-190},{120,-170}})));
  Modelica.Blocks.Interfaces.RealOutput combined_torque(
    min=-640,
    max=640,
    nominal=400) "Combined commanded torque (RL + baseline) #RLobs"
    annotation (Placement(transformation(extent={{100,-210},{120,-190}})));
  Modelica.Blocks.Interfaces.RealOutput residualRL_steeringWheelAngle(
    unit="rad",
    min=-from_deg(4)*steeringRatio,
    max=from_deg(4)*steeringRatio,
    nominal=from_deg(4)*steeringRatio) "Steering wheel angle commanded by the DRL agent #RLobs"
    annotation (Placement(transformation(extent={{100,-230},{120,-210}})));
  Modelica.Blocks.Interfaces.RealOutput residualRL_torque(
    unit="N.m",
    min=-150,
    max=150,
    nominal=150) "Torque commanded by the DRL agent #RLobs"
    annotation (Placement(transformation(extent={{100,-250},{120,-230}})));

  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{-86,-10},
            {-66,10}}),                                                                                                  iconTransformation(extent={{-20,-60},{0,-40}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{-102,-104},{-82,-84}}),
                                                                    iconTransformation(extent={{0,20},{20,40}})));

  Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-72,-72},{-50,-50}}),
        iconTransformation(extent={{-72,-72},{-50,-50}})));

public
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_yawRate
    annotation (Placement(transformation(extent={{48,94},{60,106}})));
  Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(
        transformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={200,0}),
        iconTransformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={100,0})));
  Modelica.Blocks.Math.Add calc_orientation_error(k2=-1)
    annotation (Placement(transformation(extent={{-8,-30},{12,-10}})));
  VelocityError calcVelocityError annotation (Placement(transformation(extent={{-44,54},{-24,74}})));

  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_Xposition
    annotation (Placement(transformation(extent={{48,154},{60,166}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_Yposition
    annotation (Placement(transformation(extent={{48,134},{60,146}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_sideSlipAngle
    annotation (Placement(transformation(extent={{48,114},{60,126}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_absoluteVelocity
    annotation (Placement(transformation(extent={{48,74},{60,86}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_lateralAcceleration
    annotation (Placement(transformation(extent={{48,54},{60,66}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_vehicle_longitudinalAcceleration
    annotation (Placement(transformation(extent={{48,34},{60,46}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_lateralDisplacement_error
    annotation (Placement(transformation(extent={{48,14},{60,26}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_longitudinalVelocity_error
    annotation (Placement(transformation(extent={{48,-6},{60,6}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_orientation_error
    annotation (Placement(transformation(extent={{48,-26},{60,-14}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_curvature
    annotation (Placement(transformation(extent={{48,-46},{60,-34}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_Xposition
    annotation (Placement(transformation(extent={{48,-66},{60,-54}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_Yposition
    annotation (Placement(transformation(extent={{48,-86},{60,-74}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_path_velocity
    annotation (Placement(transformation(extent={{48,-106},{60,-94}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_arcLength
    annotation (Placement(transformation(extent={{48,-126},{60,-114}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_baselineController_steeringWheelAngle
    annotation (Placement(transformation(extent={{48,-146},{60,-134}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_combined_steeringWheelAngle
    annotation (Placement(transformation(extent={{48,-166},{60,-154}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_baselineController_torque
    annotation (Placement(transformation(extent={{48,-186},{60,-174}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_combined_torque
    annotation (Placement(transformation(extent={{48,-206},{60,-194}})));

  Modelica.Blocks.Routing.RealPassThrough realPassThrough_residualRL_steeringWheelAngle
    annotation (Placement(transformation(extent={{48,-226},{60,-214}})));
  Modelica.Blocks.Routing.RealPassThrough realPassThrough_residualRL_torque
    annotation (Placement(transformation(extent={{48,-246},{60,-234}})));
  Modelica.Blocks.Interfaces.RealInput residualRL_steeringWheelAngle_in
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-60},{-100,-20}}),
        iconTransformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput residualRL_torque_in
    "Connector of Real input signal"
    annotation (Placement(transformation(extent={{-140,-102},{-100,-62}}),
        iconTransformation(extent={{-140,-102},{-100,-62}})));
equation
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{-92,-94},{-92,-26},{-56,-26},{-56,2},{-62,2},{-62,-0.1},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{-76,0},{-76,-0.1},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
      points={{-61,-61},{-99.9,-61},{-99.9,-0.1}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=-1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(orientation_error, orientation_error)
    annotation (Line(points={{110,-20},{110,-20}},
                                                 color={0,0,127}));
  connect(calc_orientation_error.u1, chassisBus.yawAngle) annotation (Line(
        points={{-10,-14},{-60,-14},{-60,-16},{-76,-16},{-76,0}}, color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(calc_orientation_error.u2, motionDemandBus.psi_path) annotation (Line(
        points={{-10,-26},{-44,-26},{-44,-61},{-61,-61}}, color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.chassisBus, chassisBus) annotation (Line(
      points={{-43.6,68.2},{-78,68.2},{-78,26},{-76,26},{-76,0}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-3,-6},{-3,-6}},
      horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.motionDemandBus, motionDemandBus) annotation (Line(
      points={{-44,60},{-54,60},{-54,-44},{-62,-44},{-62,-61},{-61,-61}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_yawRate.u, chassisBus.yawRate) annotation (
      Line(points={{46.8,100},{-16,100},{-16,60},{-14,60},{-14,0},{-76,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_torque.y, combined_torque)
    annotation (Line(points={{60.6,-200},{110,-200}}, color={0,0,127}));
  connect(realPassThrough_baselineController_torque.y,
    baselineController_torque)
    annotation (Line(points={{60.6,-180},{110,-180}}, color={0,0,127}));
  connect(realPassThrough_combined_steeringWheelAngle.y,
    combined_steeringWheelAngle)
    annotation (Line(points={{60.6,-160},{110,-160}}, color={0,0,127}));
  connect(realPassThrough_baselineController_steeringWheelAngle.y,
    baselineController_steeringWheelAngle)
    annotation (Line(points={{60.6,-140},{110,-140}}, color={0,0,127}));
  connect(realPassThrough_arcLength.y, arcLength)
    annotation (Line(points={{60.6,-120},{110,-120}}, color={0,0,127}));
  connect(realPassThrough_path_Yposition.y, path_Yposition)
    annotation (Line(points={{60.6,-80},{110,-80}}, color={0,0,127}));
  connect(realPassThrough_path_Xposition.y, path_Xposition)
    annotation (Line(points={{60.6,-60},{110,-60}}, color={0,0,127}));
  connect(realPassThrough_path_curvature.y, path_curvature)
    annotation (Line(points={{60.6,-40},{110,-40}}, color={0,0,127}));
  connect(realPassThrough_orientation_error.y, orientation_error)
    annotation (Line(points={{60.6,-20},{110,-20}}, color={0,0,127}));
  connect(realPassThrough_longitudinalVelocity_error.y,
    longitudinalVelocity_error)
    annotation (Line(points={{60.6,0},{110,0}}, color={0,0,127}));
  connect(realPassThrough_lateralDisplacement_error.y,
    lateralDisplacement_error)
    annotation (Line(points={{60.6,20},{110,20}}, color={0,0,127}));
  connect(realPassThrough_vehicle_longitudinalAcceleration.y,
    vehicle_longitudinalAcceleration)
    annotation (Line(points={{60.6,40},{110,40}}, color={0,0,127}));
  connect(realPassThrough_vehicle_lateralAcceleration.y,
    vehicle_lateralAcceleration)
    annotation (Line(points={{60.6,60},{110,60}}, color={0,0,127}));
  connect(realPassThrough_vehicle_absoluteVelocity.y, vehicle_absoluteVelocity)
    annotation (Line(points={{60.6,80},{110,80}}, color={0,0,127}));
  connect(realPassThrough_vehicle_yawRate.y, vehicle_yawRate)
    annotation (Line(points={{60.6,100},{110,100}}, color={0,0,127}));
  connect(realPassThrough_vehicle_sideSlipAngle.y, vehicle_sideSlipAngle)
    annotation (Line(points={{60.6,120},{110,120}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Yposition.y, vehicle_Yposition)
    annotation (Line(points={{60.6,140},{110,140}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Xposition.y, vehicle_Xposition)
    annotation (Line(points={{60.6,160},{110,160}}, color={0,0,127}));
  connect(realPassThrough_residualRL_steeringWheelAngle.y,
    residualRL_steeringWheelAngle)
    annotation (Line(points={{60.6,-220},{110,-220}}, color={0,0,127}));
  connect(realPassThrough_residualRL_torque.y, residualRL_torque)
    annotation (Line(points={{60.6,-240},{110,-240}}, color={0,0,127}));
  connect(realPassThrough_path_velocity.y, path_velocity)
    annotation (Line(points={{60.6,-100},{110,-100}}, color={0,0,127}));
  connect(vehicle_Xposition, fmuOutputsBus.vehicle_Xposition) annotation (Line(
        points={{110,160},{140,160},{140,0},{200,0}},             color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_Yposition, fmuOutputsBus.vehicle_Yposition) annotation (Line(
        points={{110,140},{134,140},{134,0},{200,0}},             color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_sideSlipAngle, fmuOutputsBus.vehicle_sideSlipAngle)
    annotation (Line(points={{110,120},{134,120},{134,0},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_yawRate, fmuOutputsBus.vehicle_yawRate) annotation (Line(
        points={{110,100},{110,0},{200,0}},   color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-2,-2},{-2,-5}},
      horizontalAlignment=TextAlignment.Right));
  connect(vehicle_absoluteVelocity, fmuOutputsBus.vehicle_absoluteVelocity)
    annotation (Line(points={{110,80},{138,80},{138,76},{200,76},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_lateralAcceleration, fmuOutputsBus.vehicle_lateralAcceleration)
    annotation (Line(points={{110,60},{132,60},{132,62},{200,62},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(vehicle_longitudinalAcceleration, fmuOutputsBus.vehicle_longitudinalAcceleration)
    annotation (Line(points={{110,40},{134,40},{134,0},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(lateralDisplacement_error, fmuOutputsBus.lateralDisplacement_error)
    annotation (Line(points={{110,20},{132,20},{132,22},{200,22},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(longitudinalVelocity_error, fmuOutputsBus.longitudinalVelocity_error)
    annotation (Line(points={{110,0},{142,0},{142,-2},{200,-2},{200,0}}, color={
          0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(orientation_error, fmuOutputsBus.orientation_error) annotation (Line(
        points={{110,-20},{148,-20},{148,0},{200,0}},             color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(path_curvature, fmuOutputsBus.path_curvature) annotation (Line(points={{110,-40},{146,-40},{146,0},{200,0}},
                                                            color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(path_Xposition, fmuOutputsBus.path_Xposition) annotation (Line(points={{110,-60},{110,0},{200,0}},
                                        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(path_Yposition, fmuOutputsBus.path_Yposition) annotation (Line(points={{110,-80},{138,-80},{138,-82},{200,-82},{200,0}},
                                                            color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(path_velocity, fmuOutputsBus.path_velocity) annotation (Line(points={{110,-100},{148,-100},{148,0},{200,0}},
                                                               color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(arcLength, fmuOutputsBus.arcLength) annotation (Line(points={{110,-120},{144,-120},{144,-118},{200,-118},{200,0}},
                                                             color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(baselineController_steeringWheelAngle, fmuOutputsBus.baselineController_steeringWheelAngle)
    annotation (Line(points={{110,-140},{148,-140},{148,0},{200,0}},
                    color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(combined_steeringWheelAngle, fmuOutputsBus.combined_steeringWheelAngle)
    annotation (Line(points={{110,-160},{148,-160},{148,0},{200,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(baselineController_torque, fmuOutputsBus.baselineController_torque)
    annotation (Line(points={{110,-180},{110,0},{200,0}},    color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(combined_torque, fmuOutputsBus.combined_torque) annotation (Line(
        points={{110,-200},{126,-200},{126,0},{200,0}},               color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(residualRL_steeringWheelAngle, fmuOutputsBus.residualRL_steeringWheelAngle)
    annotation (Line(points={{110,-220},{110,0},{200,0}},    color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(residualRL_torque, fmuOutputsBus.residualRL_torque) annotation (Line(
        points={{110,-240},{110,0},{200,0}},    color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(calc_orientation_error.y, realPassThrough_orientation_error.u)
    annotation (Line(points={{13,-20},{46.8,-20}}, color={0,0,127}));
  connect(realPassThrough_vehicle_Xposition.u, chassisBus.position_x)
    annotation (Line(points={{46.8,160},{16,160},{16,162},{-76,162},{-76,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_Yposition.u, chassisBus.position_y)
    annotation (Line(points={{46.8,140},{-76,140},{-76,0}}, color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_sideSlipAngle.u, chassisBus.sideSlipAngle)
    annotation (Line(points={{46.8,120},{6,120},{6,118},{-76,118},{-76,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(calcVelocityError.vehicle_absolute_velocity, realPassThrough_vehicle_absoluteVelocity.u)
    annotation (Line(points={{-23,70},{42,70},{42,80},{46.8,80}}, color={0,0,127}));
  connect(realPassThrough_vehicle_lateralAcceleration.u, chassisBus.lateralAcceleration)
    annotation (Line(points={{46.8,60},{-30,60},{-30,0},{-76,0}}, color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_vehicle_longitudinalAcceleration.u, chassisBus.longitudinalAcceleration)
    annotation (Line(points={{46.8,40},{24,40},{24,42},{-38,42},{-38,0},{-76,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_lateralDisplacement_error.u, motionDemandBus.e_lat)
    annotation (Line(points={{46.8,20},{30,20},{30,22},{-58,22},{-58,-61},{-61,-61}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_longitudinalVelocity_error.u, calcVelocityError.longitudinal_velocity_error)
    annotation (Line(points={{46.8,0},{36,0},{36,4},{4,4},{4,64},{-23,64}},
        color={0,0,127}));
  connect(realPassThrough_path_curvature.u, motionDemandBus.kappa_path)
    annotation (Line(points={{46.8,-40},{-42,-40},{-42,-78},{-61,-78},{-61,-61}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_Xposition.u, motionDemandBus.x_path) annotation
    (Line(points={{46.8,-60},{-40,-60},{-40,-64},{-46,-64},{-46,-61},{-61,-61}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_Yposition.u, motionDemandBus.y_path) annotation
    (Line(points={{46.8,-80},{-61,-80},{-61,-61}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_path_velocity.u, motionDemandBus.v_path) annotation (
      Line(points={{46.8,-100},{-61,-100},{-61,-61}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_arcLength.u, motionDemandBus.arc_length) annotation (
      Line(points={{46.8,-120},{14,-120},{14,-118},{-61,-118},{-61,-61}}, color
        ={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_baselineController_steeringWheelAngle.u,
    chassisControlBus.baseline_steering) annotation (Line(points={{46.8,-140},{-92,
          -140},{-92,-94}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_steeringWheelAngle.u, chassisControlBus.steeringWheelAngle)
    annotation (Line(points={{46.8,-160},{-108,-160},{-108,-94},{-92,-94}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_baselineController_torque.u, chassisControlBus.baseline_torque)
    annotation (Line(points={{46.8,-180},{36,-180},{36,-122},{-74,-122},{-74,-94},
          {-92,-94}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(realPassThrough_combined_torque.u, chassisControlBus.torque)
    annotation (Line(points={{46.8,-200},{32,-200},{32,-124},{-90,-124},{-90,-108},
          {-92,-108},{-92,-94}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(residualRL_steeringWheelAngle_in,
    realPassThrough_residualRL_steeringWheelAngle.u) annotation (Line(points={{-120,
          -40},{30,-40},{30,-220},{46.8,-220}},   color={0,0,127}));
  connect(residualRL_torque_in, realPassThrough_residualRL_torque.u)
    annotation (Line(points={{-120,-82},{-36,-82},{-36,-240},{46.8,-240}},
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
