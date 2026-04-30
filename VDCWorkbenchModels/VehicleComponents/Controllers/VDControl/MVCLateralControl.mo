within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model MVCLateralControl "Inversion based vehicle dynamics control"
  extends BaseClasses.BaseVDC(
    filePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt.mat"),
    pathName="path");

  parameter Modelica.Units.SI.AngularVelocity omega_n = 2*3.14*0.5 "Desired bandwidth of v-controller" annotation(Dialog(group="Velocity controller parameters"));
  parameter Real vctrl_Kp = 2*0.7*omega_n*m*car_r "P-gain for v-control" annotation(Dialog(group="Velocity controller parameters"));
  parameter Real vctrl_Ki = omega_n*omega_n*m*car_r "I-gain for v-control" annotation(Dialog(group="Velocity controller parameters"));
  parameter Modelica.Units.SI.Torque vctrl_TorqueMax = Torque_max_frontMotor+2*Torque_max_rearMotor
    "Maximum wheel torque" annotation(Dialog(group="Velocity controller parameters"));

  Components.SingleTrackModel_SS singleTrackModel_SS(
    m=m,
    lf=lf,
    lr=lr,
    J=J,
    cf=cf,
    cr=cr,
    ayMax=5) annotation (Placement(transformation(extent={{10,30},{30,50}})));
  Modelica.Blocks.Math.Gain steeringGain(k=steer_gain) annotation (Placement(transformation(extent={{50,40},{70,60}})));
  Modelica.Blocks.Continuous.LimPID v_controller(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=vctrl_Kp,
    Ti=vctrl_Kp/vctrl_Ki,
    yMax=vctrl_TorqueMax)
    annotation (Placement(transformation(extent={{-30,10},{-10,-10}})));
  Components.TorqueVectoring torqueVectoring(
    wheel_r=car_r,
    track_width=track_width,
    Torque_max_frontMotor=Torque_max_frontMotor,
    Torque_max_rearMotor=Torque_max_rearMotor)
    annotation (Placement(transformation(extent={{10,-50},{30,-30}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{-52,-6},{-40,6}})));
  Modelica.Blocks.Nonlinear.Limiter v_scl_lim(uMax=1, uMin=0)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Modelica.Blocks.Sources.CombiTimeTable parametricPath(
    tableOnFile=true,
    tableName=pathName,
    fileName=filePath,
    verboseRead=true,
    columns={1,2,3,4,5,6,7})
    "Parametric path description with extra time calculated from reference velocity in first column"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{-30,60},{-10,80}}), iconTransformation(extent={{-20,-60},{0,-40}})));
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{70,-10},{90,10}}),iconTransformation(extent={{0,20},{20,40}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusFM
    annotation (Placement(transformation(extent={{70,-30},{90,-10}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRL
    annotation (Placement(transformation(extent={{70,-50},{90,-30}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
  VDCWorkbenchModels.Utilities.Interfaces.ElectricDriveControlBus electricMotorControlBusRR
    annotation (Placement(transformation(extent={{70,-70},{90,-50}}),
        iconTransformation(extent={{-130,-30},{-110,-10}})));
  Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0,
        origin={-58,90}),
        iconTransformation(
        extent={{-16,-17},{16,17}},
        rotation=270,
        origin={-51,0})));
equation
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{80,0},{90,0},{90,-80},{0.1,-80},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(v_controller.y, chassisControlBus.torque) annotation (Line(points={{-9,0},{80,0}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(steeringGain.y, chassisControlBus.steeringWheelAngle) annotation (Line(points={{71,50},{80,50},{80,0}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(singleTrackModel_SS.steer_f, steeringGain.u)
    annotation (Line(points={{31,46},{40,46},{40,50},{48,50}}, color={0,0,127}));
  connect(v_controller.y, torqueVectoring.Torque) annotation (Line(points={{-9,0},{0,0},{0,-34},{8,-34}},
        color={0,0,127}));
  connect(TV_ratio, torqueVectoring.alpha_TV) annotation (Line(points={{-120,-82},{-90,-82},{-90,-46},{8,-46}},
        color={0,0,127}));
  connect(torqueVectoring.Mz_TorqueVectoring, singleTrackModel_SS.Mz_TorqueVect)
    annotation (Line(points={{31,-38},{40,-38},{40,20},{0,20},{0,34},{8,34}},
        color={0,0,127}));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{-20,70},{-20,90},{90,90},{90,-80},{0.1,-80},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(v_controller.u_m, chassisBus.longitudinalVelocity) annotation (Line(points={{-20,12},{-20,70}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{3,3},{3,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(singleTrackModel_SS.v, chassisBus.longitudinalVelocity) annotation (Line(points={{8,40},{-20,40},{-20,70}}, color = {0, 0, 127}));
  connect(product1.y, v_controller.u_s)
    annotation (Line(points={{-39.4,0},{-32,0}}, color={0,0,127}));
  connect(v_scl, v_scl_lim.u)
    annotation (Line(points={{-120,0},{-92,0}}, color={0,0,127}));
  connect(v_scl_lim.y, product1.u2) annotation (Line(points={{-69,0},{-64,0},{-64,-3.6},{-53.2,-3.6}},
        color={0,0,127}));
  connect(FrontRear_ratio, torqueVectoring.alpha_FrontRear) annotation (Line(
        points={{-120,-40},{8,-40}},
        color={0,0,127}));
  connect(electricMotorControlBusFM, controlBus.electricMotorControlBusFM) annotation (Line(
      points={{80,-20},{90,-20},{90,-80},{0,-80},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(electricMotorControlBusRL, controlBus.electricMotorControlBusRL) annotation (Line(
      points={{80,-40},{90,-40},{90,-80},{0,-80},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(electricMotorControlBusRR, controlBus.electricMotorControlBusRR) annotation (Line(
      points={{80,-60},{90,-60},{90,-80},{0,-80},{0,-99.9},{0.1,-99.9}},
      color={255,204,51},
      thickness=0.5));
  connect(torqueVectoring.torque_dem_front, electricMotorControlBusFM.torque) annotation (Line(points={{31,-34},{70,-34},{70,-20},{80,-20}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearLeft, electricMotorControlBusRL.torque) annotation (Line(points={{31,-42},{70,-42},{70,-40},{80,-40}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(torqueVectoring.torque_dem_rearRight, electricMotorControlBusRR.torque) annotation (Line(points={{31,-46},{70,-46},{70,-60},{80,-60}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}},
      horizontalAlignment=TextAlignment.Left));
  connect(parametricPath.y[6], product1.u1) annotation (Line(points={{-79,70},{-78,70},{-78,68},{-60,68},{-60,3.6},{-53.2,3.6}}, color={0,0,127}));
  connect(parametricPath.y[7], singleTrackModel_SS.curvature) annotation (Line(
        points={{-79,70},{0,70},{0,46},{8,46}}, color={0,0,127}));
  connect(controlBus.motionDemandBus,motionDemandBus)  annotation (Line(
      points={{0.1,-99.9},{0.1,-80},{90,-80},{90,90},{-58,90}},
      color={255,204,51},
      thickness=0.5));
  connect(parametricPath.y[7], motionDemandBus.kappa_path) annotation (Line(
        points={{-79,70},{-58,70},{-58,90}}, color={0,0,127}));
  connect(parametricPath.y[6], motionDemandBus.v_path) annotation (Line(points={{-79,70},{-78,70},{-78,68},{-60,68},{-60,90},{-58,90}},
        color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Text(
          extent={{-100,-60},{100,-90}},
          textColor={0,0,0},
          textString="MVC-Lat")}),
    Documentation(
      info="<html>
<p>
A longitudinal and lateral control algorithm to control planar behavior of a&nbsp;vehicle.
To simplify the design, it is assumed that the vehicle acceleration is well below the wheels&apos;
adhesion limit. This allows to independently handle the longitudinal and lateral motion of
the vehicle in the design process.
</p>
<p>
The <em>longitudinal controller</em> <code>v_controller</code> aims to follow the reference velocity
<var>v</var> by manipulating the overall traction torque&nbsp;&tau;. A&nbsp;PI algorithm is
used to achieve this goal, with the proportional and integral gains, <code>vctrl_Kp</code>
and <code>vctrl_Ki</code> respectively.
The longitudinal controller also contains two additional mechanisms:
</p>
<ul>
  <li>
    saturation of the control signal in order to match the minimum/maximum force that
    the drivetrain can develop,
  </li>
  <li>
    anti-windup mechanism to limit the accumulation of (integral) error when the control
    actuation is saturated.
  </li>
</ul>
<p>
The overall traction torque&nbsp;&tau; generated by the controller is ultimately mapped
into reference motor torques via the <code>controlBus</code>.
</p>
<p>
The <em>lateral control algorithm</em> employs a&nbsp;model inversion approach to track
the reference curvature provided by the reference path <code>parametricPath</code>.
See <a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components.SingleTrackModel_SS\">inverse single-track model</a>
for further details.
</p>
</html>"));
end MVCLateralControl;
