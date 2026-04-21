within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model RearAxleStanleyControllerTD
  extends VDControl.BaseClasses.BaseSubBusses;
  // TIPI Parameters
  parameter Real e_long_gain = 80;
  parameter Real s_start = 0;
  parameter Real t_ff = 0.0;

  parameter Real k = 5;
  parameter Real v_eps = 0.1;
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;
  parameter Real deltaMax = 0.3;
  parameter Real K_vctrl = 0.5;
  parameter Real vctrl_TorqueMax = 0.3;

  parameter Real Ts = 0.05;

  parameter Real m = 7.151;
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 150;

  VDControl.TimeIndependetPathInterpolation.RearAxleTIPI rearAxleTIPI(
    e_long_gain=e_long_gain,
    s_start=s_start,
    t_ff=t_ff,
    lr=lr) annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  VDControl.StanleyBased.RearAxleStanleyControlTD rearAxleStanleyControl(
    K=k,
    v_eps=v_eps,
    k_d_yaw=k_d_yaw,
    k_d_steer=k_d_steer,
    deltaMax=deltaMax,
    K_vctrl=K_vctrl,
    vctrl_TorqueMax=vctrl_TorqueMax,
    Ts=Ts,
    m=m,
    lf=lf,
    lr=lr,
    C_Tire=C_Tire) annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
equation
  connect(const.y, rearAxleTIPI.v_scl) annotation (Line(points={{-59,30},{-52,30},
          {-52,36},{-42,36}}, color={0,0,127}));
  connect(rearAxleTIPI.controlBus, controlBus) annotation (Line(
      points={{-20,30},{0,30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(rearAxleStanleyControl.controlBus, controlBus) annotation (Line(
      points={{-20,-30},{0,-30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(rearAxleStanleyControl.delta, chassisControlBus.steeringWheelAngle)
    annotation (Line(points={{-18.7,-22.1},{56,-22.1},{56,20},{80,20}},   color
        ={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
  connect(rearAxleStanleyControl.torque, electricMotorControlBus.torque) annotation (
      Line(points={{-18.8,-26},{60,-26},{60,-20},{80,-20}},
         color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{2,2},{2,5}},
      horizontalAlignment=TextAlignment.Left));
end RearAxleStanleyControllerTD;
