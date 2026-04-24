within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model MiniAFMGeoPFC "Geometry based path following control for the miniAFM"
  extends BaseClasses.BaseSubBusses(
    m=7.151,
    lf=0.1805,
    lr=0.1805,
    track_width=0.265,
    J=1/12 * 7.151 * (0.265^2 + 0.361^2),
    car_r=0.0525,
    cf=30e3,
    cr=30e3);

  parameter Real e_long_gain=80 "TIPI controller gain to force e_long to 0" annotation(Dialog(group="TIPI controller parameters"));
  parameter Real e_y_ref=0 "TIPI eccentric parameter to distinguish road side" annotation(Dialog(group="TIPI controller parameters"));

  parameter Real lambda_eLat=0.1 "Max. lateral deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));
  parameter Real lambda_del_psi=0.1 "Max. yaw deviation for motion demand calculation" annotation(Dialog(group="Motion demand controller parameters"));

  VDControl.TimeIndependetPathInterpolation.CoGTIPI tIPI(
    e_long_gain=e_long_gain,
    filePath=ModelicaServices.ExternalReferences.loadResource(
        "modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat"),
    maxArcLength=22.737000000000002,
    pathName="path") "Time-independent path interpolation"
    annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  VDControl.GeoPFC.MotionDemand motionDemand(
    lambda_eLat=lambda_eLat,
    lambda_del_psi=lambda_del_psi,
    e_y_ref=e_y_ref) "Calculate motion demand"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  VDControl.GeoPFC.ControlAllocation controlAllocation(
    lf=lf,
    lr=lr,
    maxTau=0.5,
    Kspeedctrl=0.5) "Motion control allocation"
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Modelica.Blocks.Sources.Constant const(k=1.0)
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
equation
  connect(tIPI.controlBus, controlBus) annotation (Line(
      points={{-20,30},{0,30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(motionDemand.controlBus, controlBus) annotation (Line(
      points={{-20,0},{0,0},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(controlAllocation.controlBus, controlBus) annotation (Line(
      points={{-20,-30},{0,-30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(controlAllocation.delta, chassisControlBus.steeringWheelAngle) annotation (
      Line(points={{-19,-22},{56,-22},{56,20},{80,20}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(controlAllocation.torque, electricMotorControlBus.torque) annotation (
      Line(points={{-19,-26},{60,-26},{60,-20},{80,-20}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(const.y, tIPI.v_scl) annotation (Line(points={{-59,30},{-52,30},{-52,36},{-42,36}}, color={0,0,127}));
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
end MiniAFMGeoPFC;
