within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
model PredStanleyControllerTD
  extends VDControl.BaseClasses.BaseSubBusses(
    m = 7.151,
    lf = 0.1805,
    lr = 0.1805);

  // TIPI Parameters
  parameter Real e_long_gain = 80 "TIPI Controller gain to force e_long to 0";
  parameter Modelica.Units.SI.Position s_start = 0 "Arc length value at vehicle start position";
  parameter Real t_ff = 0.0;

  parameter Real k = 5 "Stanley gain";
  parameter Modelica.Units.SI.Velocity v_eps = 0.1  "Small velocity to avoid division by zero";
  parameter Real k_d_yaw = 0.14 "Factor for yaw rate related damping";
  parameter Real k_d_steer = 0.0 "Factor penalizing rate of steering angle change";
  parameter Modelica.Units.SI.Angle deltaMax = 0.3 "Steering saturation";

  parameter Boolean use_prediction = true "= true, if prediction shall be activated" annotation (Dialog(tab="Advanced"));
  parameter Integer N = 4 "Number of prediction steps"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));
  parameter Modelica.Units.SI.Time dt = Ts "Predicition time step"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));
  parameter Real weights[N+1] = {0.4, 0.2, 0.2, 0.1, 0.1} "Weighting of predicted steering command"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));

  parameter Real K_vctr = 0.5 "Gain of torque control" annotation (Dialog(group="Torque controller"));
  parameter Modelica.Units.SI.Torque vctr_TorqueMax = 0.3 "Torque limit" annotation (Dialog(group="Torque controller"));
  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation"
    annotation(Dialog(tab="Vehicle parameters"));

  VDControl.TimeIndependetPathInterpolation.FrontAxleTIPI tIPI(
    e_long_gain=e_long_gain,
    s_start=s_start,
    lf=lf) "Time-independent path interpolation" annotation (Placement(transformation(extent={{-40,20},{-20,40}})));
  VDControl.StanleyBased.PredictiveStanleyControlTD stanleyControl(
    k=k,
    v_eps=v_eps,
    k_d_yaw=k_d_yaw,
    k_d_steer=k_d_steer,
    deltaMax=deltaMax,
    K_vctr=K_vctr,
    vctr_TorqueMax=vctr_TorqueMax,
    Ts=Ts,
    m=m,
    lf=lf,
    lr=lr,
    C_Tire=C_Tire,
    final use_prediction=use_prediction,
    final weights=weights,
    final N=N,
    final dt=dt) annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
equation
  connect(const.y, tIPI.v_scl) annotation (Line(points={{-59,30},{-52,30},{-52,36},{-42,36}}, color={0,0,127}));
  connect(tIPI.controlBus, controlBus) annotation (Line(
      points={{-20,30},{0,30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
  connect(stanleyControl.delta, chassisControlBus.steeringWheelAngle) annotation (
      Line(
        points={{-18.7,-22.1},{56,-22.1},{56,20},{80,20}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(stanleyControl.torque, electricMotorControlBus.torque) annotation (
      Line(
        points={{-18.8,-26},{60,-26},{60,-20},{80,-20}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{2,2},{2,5}},
        horizontalAlignment=TextAlignment.Left));
  connect(stanleyControl.controlBus, controlBus) annotation (Line(
      points={{-20,-30},{0,-30},{0,-100}},
      color={255,204,51},
      thickness=0.5));
end PredStanleyControllerTD;
