within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model VDCWorkbenchControl "Geometry based path following control"
  extends BaseClasses.BaseVDC;

  parameter Real e_long_gain=80 "TIPI controller gain to force e_long to 0" annotation(Dialog(group="TIPI controller parameters"));
  parameter Real e_y_ref=0 "TIPI eccentric parameter to distinguish road side" annotation(Dialog(group="TIPI controller parameters"));

  parameter Real lambda_eLat=10 "Max. lateral deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));
  parameter Real lambda_del_psi=20 "Max. yaw deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));

  parameter Real vctrl_Kp = 1000 "P-gain for v-control" annotation(Dialog(group="Velocity controller parameters"));
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = Torque_max_frontMotor+2*Torque_max_rearMotor
    "Maximum total torque" annotation(Dialog(group="Velocity controller parameters"));

  Modelica.Blocks.Math.Gain steeringGain(k=steer_gain) annotation (Placement(transformation(extent={{40,40},{60,60}})));
  Components.TorqueVectoring torqueVectoring(
    wheel_r=car_r,
    track_width=track_width,
    Torque_max_frontMotor=Torque_max_frontMotor,
    Torque_max_rearMotor=Torque_max_rearMotor)
    annotation (Placement(transformation(extent={{20,-80},{40,-60}})));
  Modelica.Blocks.Nonlinear.Limiter v_scl_lim(uMax=1, uMin=0)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  GeoPFC.TimeIndependentPathInterpolation tIPI_bus(e_long_gain=e_long_gain)
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  GeoPFC.MotionDemand calculate_Motion_Demand(
    lambda_eLat=lambda_eLat,
    lambda_del_psi=lambda_del_psi,
    e_y_ref=e_y_ref) annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  GeoPFC.ControlAllocation calculate_Control_Allocation(
    lf=lf,
    lr=lr,
    maxTau=vctrl_TorqueMax,
    Kspeedctrl=vctrl_Kp) annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));

protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{70,0},{90,20}}),
        iconTransformation(extent={{-20,-60},{0,-40}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{70,20},{90,40}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusFM
    annotation (Placement(transformation(extent={{70,-50},{90,-30}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRL
    annotation (Placement(transformation(extent={{70,-70},{90,-50}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRR
    annotation (Placement(transformation(extent={{70,-90},{90,-70}})));
equation
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{80,30},{80,-90},{0,-90},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(steeringGain.y, chassisControlBus.steeringWheelAngle) annotation (Line(points={{61,50},{80,50},{80,30}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(TV_ratio, torqueVectoring.alpha_TV) annotation (Line(points={{-120,-82},{-90,-82},{-90,-76},{18,-76}},
        color={0,0,127}));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{80,10},{80,-90},{0.1,-90},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(v_scl, v_scl_lim.u)
    annotation (Line(points={{-120,0},{-92,0}},
        color={0,0,127}));
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
  connect(torqueVectoring.torque_dem_front, electricMotorControlBusFM.torque) annotation (Line(points={{41,-64},{60,-64},{60,-40},{80,-40}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearLeft, electricMotorControlBusRL.torque) annotation (Line(points={{41,-72},{62,-72},{62,-60},{80,-60}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearRight, electricMotorControlBusRR.torque) annotation (Line(points={{41,-76},{64,-76},{64,-80},{80,-80}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(tIPI_bus.controlBus, controlBus) annotation (Line(
      points={{-20,30},{-14,30},{-14,-90},{0,-90},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(calculate_Motion_Demand.controlBus, controlBus) annotation (Line(
      points={{-20,0},{-14,0},{-14,-90},{0,-90},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(calculate_Control_Allocation.delta, steeringGain.u) annotation (Line(
        points={{-19,-22},{0,-22},{0,50},{38,50}}, color={0,0,127}));
  connect(calculate_Control_Allocation.torque, torqueVectoring.Torque)
    annotation (Line(points={{-19,-26},{0,-26},{0,-64},{18,-64}}, color={0,0,127}));
  connect(calculate_Control_Allocation.controlBus, controlBus) annotation (Line(
      points={{-20,-30},{-14,-30},{-14,-90},{0,-90},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(v_scl_lim.y, tIPI_bus.v_scl) annotation (Line(points={{-69,0},{-60,0},{-60,36},{-42,36}},
        color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-100,-60},{100,-90}},
          textColor={0,0,0},
          textString="VDC-Geo")}),
    Documentation(info="<html>
<p>
The complete geometric path-following control consisting of three main modules.
</p>
<ul>
  <li>
    The top module (yellow triangle) includes the time independent path interpolation (TIPI)
    implementation for determining path with the readout of the pre-planned parametric path
    description.
  </li>
  <li>
    In the second module, MotionDemandAllocation, a&nbsp;high-level motion setpoints for
    the vehicle are planned, e.g. curvature that the vehicle should follow in order to
    track the reference path.
  </li>
  <li>
    The control allocation and the torque vectoring map the motion demand into the final
    actuation setpoints, such as traction motor torques and steering angle
  </li>
</ul>
</html>"));
end VDCWorkbenchControl;
