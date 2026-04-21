within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model VDCWorkbenchControl_RL "Geometry based path following control for residual reinforcement learning"
  extends BaseClasses.BaseVDC(
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt_TIPI.mat"),
    pathName="path_TIPI");

  parameter Real e_long_gain=80 "TIPI controller gain to force e_long to 0" annotation(Dialog(group="TIPI controller parameters"));
  parameter Real e_y_ref=0 "TIPI eccentric parameter to distinguish road side" annotation(Dialog(group="TIPI controller parameters"));

  parameter Real lambda_eLat=10 "Max. lateral deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));
  parameter Real lambda_del_psi=20 "Max. yaw deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));

  parameter Real vctrl_Kp = 1000 "P-gain for v-control" annotation(Dialog(group="Velocity controller parameters"));
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = Torque_max_frontMotor+2*Torque_max_rearMotor
    "Maximum total torque" annotation(Dialog(group="Velocity controller parameters"));

  parameter Real v_scale=1.0 "Scaling factor for the reference path velocity" annotation(Dialog(group="TIPI controller parameters"));

  Modelica.Blocks.Math.Gain steeringGain(k=steer_gain) annotation (Placement(transformation(extent={{30,40},{50,60}})));

  Components.TorqueVectoring torqueVectoring(
    wheel_r=car_r,
    track_width=track_width,
    Torque_max_frontMotor=Torque_max_frontMotor,
    Torque_max_rearMotor=Torque_max_rearMotor)
    annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
  Modelica.Blocks.Math.Gain v_scl_scaling(k=v_scale) "Scaling of the path velocity" annotation (Placement(transformation(extent={{-92,-6},{-80,6}})));
  Modelica.Blocks.Nonlinear.Limiter v_scl_lim(uMax=1, uMin=0)
    annotation (Placement(transformation(extent={{-72,-6},{-60,6}})));

  TimeIndependetPathInterpolation.CoGTIPI tIPI_bus(
    e_long_gain=e_long_gain,
    filePath=filePath,
    pathName=pathName)
    annotation (Placement(transformation(extent={{-30,30},{-10,50}})));
  GeoPFC.MotionDemand calculate_Motion_Demand(
    lambda_eLat=lambda_eLat,
    lambda_del_psi=lambda_del_psi,
    e_y_ref=e_y_ref)
    annotation (Placement(transformation(extent={{-30,0},{-10,20}})));
  GeoPFC.ControlAllocationRL calculate_Control_Allocation(
    lf=lf,
    lr=lr,
    maxTau=vctrl_TorqueMax,
    Kspeedctrl=vctrl_Kp)
    annotation (Placement(transformation(extent={{-30,-30},{-10,-10}})));

  Modelica.Blocks.Interfaces.RealInput addRL_FrontSteering_in
    "Connector of Real input signal 1"
    annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
  Modelica.Blocks.Interfaces.RealInput addRL_TotalTorque_in
    annotation (Placement(transformation(extent={{-140,20},{-100,60}}),
        iconTransformation(extent={{-140,20},{-100,60}})));

  Modelica.Blocks.Math.Gain transform_to_steeringAngle1(k=steer_gain)
    annotation (Placement(transformation(extent={{30,10},{50,30}})));
  Modelica.Blocks.Math.Gain transform_to_steeringAngle(k=1/steer_gain)
    annotation (Placement(transformation(extent={{-90,70},{-70,90}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{70,-18},{90,2}}),     iconTransformation(extent={{-20,-60},{0,-40}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{70,2},{90,22}}),  iconTransformation(extent={{0,20},{20,40}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusFM
    annotation (Placement(transformation(extent={{70,-50},{90,-30}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRL
    annotation (Placement(transformation(extent={{70,-70},{90,-50}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRR
    annotation (Placement(transformation(extent={{70,-90},{90,-70}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
equation
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{80,12},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(steeringGain.y, chassisControlBus.steeringWheelAngle) annotation (Line(points={{51,50},{80,50},{80,12}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(TV_ratio, torqueVectoring.alpha_TV) annotation (Line(points={{-120,-80},{-90,-80},{-90,-76},{18,-76}},
        color={0,0,127}));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{80,-8},{80,-90},{0.1,-90},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(FrontRear_ratio, torqueVectoring.alpha_FrontRear) annotation (Line(
        points={{-120,-40},{-90,-40},{-90,-70},{18,-70}},
        color={0,0,127}));
  connect(electricMotorControlBusFM, controlBus.electricMotorControlBusFM) annotation (Line(
      points={{80,-40},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(electricMotorControlBusRL, controlBus.electricMotorControlBusRL) annotation (Line(
      points={{80,-60},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(electricMotorControlBusRR, controlBus.electricMotorControlBusRR) annotation (Line(
      points={{80,-80},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(torqueVectoring.torque_dem_front, electricMotorControlBusFM.torque) annotation (Line(points={{41,-64},{60,-64},{60,-40},{80,-40}},   color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,2}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearLeft, electricMotorControlBusRL.torque) annotation (Line(points={{41,-72},{62,-72},{62,-60},{80,-60}},   color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,2}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearRight, electricMotorControlBusRR.torque) annotation (Line(points={{41,-76},{62,-76},{62,-80},{80,-80}},   color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,2}},
      horizontalAlignment=TextAlignment.Left));
  connect(tIPI_bus.controlBus, controlBus) annotation (Line(
      points={{-10,40},{0,40},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(calculate_Motion_Demand.controlBus, controlBus) annotation (Line(
      points={{-10,10},{0,10},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(calculate_Control_Allocation.controlBus, controlBus) annotation (Line(
      points={{-10,-20},{0,-20},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(v_scl_lim.y, tIPI_bus.v_scl) annotation (Line(points={{-59.4,0},{-40,0},{-40,46},{-32,46}},
        color={0,0,127}));
  connect(transform_to_steeringAngle1.y, chassisControlBus.baseline_steering)
    annotation (Line(points={{51,20},{60,20},{60,14},{78,14},{78,12},{80,12}},
                                               color={0,0,127}),
      Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(addRL_FrontSteering_in, transform_to_steeringAngle.u)
    annotation (Line(points={{-120,80},{-92,80}}, color={0,0,127}));
  connect(transform_to_steeringAngle.y, calculate_Control_Allocation.addRL_FrontSteering_in)
    annotation (Line(points={{-69,80},{-46,80},{-46,-14},{-32,-14}}, color={0,0,127}));
  connect(addRL_TotalTorque_in, calculate_Control_Allocation.addRL_TotalTorque_in)
    annotation (Line(points={{-120,40},{-50,40},{-50,-20},{-32,-20}}, color={0,0,127}));
  connect(torqueVectoring.Torque, calculate_Control_Allocation.torque)
    annotation (Line(points={{18,-64},{8,-64},{8,-16},{-9,-16}},  color={0,0,127}));
  connect(calculate_Control_Allocation.baseline_delta, transform_to_steeringAngle1.u) annotation (Line(points={{-9,-25.4},{20,-25.4},{20,20},{28,20}},
        color={0,0,127}));
  connect(calculate_Control_Allocation.delta, steeringGain.u) annotation (Line(
        points={{-9,-12},{18,-12},{18,50},{28,50}},
        color={0,0,127}));
  connect(calculate_Control_Allocation.baseline_torque, chassisControlBus.baseline_torque)
    annotation (Line(points={{-9,-28},{60,-28},{60,10},{78,10},{78,12},{80,12}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(calculate_Control_Allocation.torque, chassisControlBus.torque)
    annotation (
      Line(
        points={{-9,-16},{58,-16},{58,12},{80,12}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(v_scl, v_scl_scaling.u) annotation (Line(points={{-120,0},{-93.2,0}}, color={0,0,127}));
  connect(v_scl_scaling.y, v_scl_lim.u) annotation (Line(points={{-79.4,0},{-73.2,0}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={Text(
          extent={{-88,-44},{92,-96}},
          textColor={28,108,200},
          textString="VDC-Geo"), Text(
          extent={{-96,94},{-40,36}},
          textColor={238,46,47},
          textString="RL")}),
    Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VDCWorkbenchControl_RL;
