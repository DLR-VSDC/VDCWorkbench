within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model PredictiveStanleyControlTD "Time-discrete predictive Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  parameter Real k = 5 "Stanley gain";
  parameter Modelica.Units.SI.Velocity v_eps = 0.1  "Small velocity to avoid division by zero";
  parameter Real k_d_yaw = 0.14 "Factor for yaw rate related damping";
  parameter Real k_d_steer = 0.0 "Factor penalizing rate of steering angle change";

  parameter Modelica.Units.SI.Angle deltaMax = 0.3 "Steering saturation";

  parameter Real K_vctr = 0.5 "P gain of velocity controller" annotation (Dialog(group="Torque controller"));
  parameter Modelica.Units.SI.Torque vctr_TorqueMax = 0.3 "Torque limit" annotation (Dialog(group="Torque controller"));

  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

  parameter Modelica.Units.SI.Mass m = 7.151 "Vehicle mass" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lf = 0.1805 "Distance of CoG to front axle" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lr = 0.1805 "Distance of CoG to rear axle" annotation(Dialog(group="Vehicle parameters"));
  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation" annotation(Dialog(group="Vehicle parameters"));

  parameter Boolean use_prediction=true "= true, if prediction shall be activated" annotation (Dialog(tab="Advanced"));
  parameter Integer N = 4 "Number of prediction steps"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));
  parameter Modelica.Units.SI.Time dt = Ts "Predicition time step"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));
  parameter Real weights[N+1] = {0.4, 0.2, 0.2, 0.1, 0.1} "Weighting of predicted steering command"
    annotation(Dialog(enable=use_prediction, group="Prediction horizon (if use_prediction = true)"));

  // ===================== Path data =====================
  parameter Integer nPath = 4999 "Number of path points";
  parameter String FilePath = ModelicaServices.ExternalReferences.loadResource(
    "modelica://VDCWorkbenchModels/Resources/Maps/RacetrackMini.mat")
    "File where path table information is stored in table 'path'";
  parameter String pathName = "path" "Table name in .mat file";
  parameter Real pathData[nPath, 6] = Modelica.Utilities.Streams.readRealMatrix(
    FilePath, pathName, nPath, 6);
  parameter Real maxArc = pathData[nPath, 1];

public
  Real x_front, y_front;
  Real e_lat;
  Real yawRate_path;
  Real delta_k0;
  Real delta_raw;
  Modelica.Units.SI.Angle e_psi;
  Modelica.Units.SI.Angle delta_yaw;
  Modelica.Units.SI.Angle delta_steer;
  Modelica.Units.SI.Angle dpsi;
  Real psi_ss;

  discrete Real delta_km1(start=0);
  discrete Real delta_km2(start=0);

  Real inputs[2];
  Real vehStates[6];

protected
  Modelica.Blocks.Interfaces.RealOutput arcLength annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,-80})));
  Modelica.Blocks.Interfaces.RealOutput vveh_lat "Lateral velocity of vehicle" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={30,-80})));

algorithm
  when sample(0, Ts) then
    // set coordinates to center of front axle
    x_front :=xveh + lf*cos(psiveh);
    y_front :=yveh + lf*sin(psiveh);

    // calc errors
    e_lat :=-Modelica.Math.sin(psi_path)*(x_path - x_front) + Modelica.Math.cos(
       psi_path)*(y_path - y_front);

    // Slip angle compensation
    yawRate_path := vveh_long*kappa_path;
    psi_ss := (m/(C_Tire*(1 + lf/lr)))*vveh_long*yawRate_path;
    dpsi := psi_path - psiveh - psi_ss;
    e_psi := Modelica.Math.atan2(Modelica.Math.sin(dpsi), Modelica.Math.cos(dpsi));

    // yaw rate damping
    delta_yaw := k_d_yaw*(yawRate_path - yaw_rate);

    // steer response damping
    delta_steer := k_d_steer * (delta_km1 - delta_km2);

    // Stanley control law
    delta_k0 := e_psi + Modelica.Math.atan(k*e_lat/(vveh_long + v_eps)) + delta_yaw + delta_steer;
    delta_k0 := min(max(delta_k0, -deltaMax), deltaMax);

    torque := min(vctr_TorqueMax, max(-vctr_TorqueMax, K_vctr*(v_path - vveh_long)));

    delta_km2 := delta_km1;
    delta_km1 := delta;

    // prediction part of model
    if use_prediction then
      vehStates := {xveh,yveh,psiveh,vveh_long,vveh_lat,yaw_rate};
      inputs :={delta_k0, torque};
      delta_raw :=weights[1]*delta_k0 + weights[2:(N + 1)]*Components.prediction(
        vehStates,
        arcLength,
        inputs,
        dt,
        N,
        pathData,
        delta_k0,
        delta_km1);
    else
      delta_raw := delta_k0;
    end if;

    delta :=min(max(delta_raw, -deltaMax), deltaMax);

  end when;

equation
  connect(arcLength, motionDemandBus.arc_length) annotation (Line(points={{-30,-80},{10,-80},{10,30}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(vveh_lat, chassisBus.lateralVelocity) annotation (Line(points={{30,-80},{70,-80},{70,30}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  annotation (Icon(graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={244,125,35},
          fillPattern=FillPattern.Forward,
          pattern=LinePattern.DashDot),
        Text(
          extent={{-100,60},{100,0}},
          textColor={255,255,255},
          textString="predictive
Stanley"),
        Line(
          points={{-80,-80},{-80,-40},{-30,-40},{-30,-20},{20,-20},{20,-60},{70,-60},{70,-10}},
          color={255,255,255},
          pattern=LinePattern.Dot),
        Ellipse(
          extent={{-86,-34},{-74,-46}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-36,-14},{-24,-26}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{14,-54},{26,-66}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}));
end PredictiveStanleyControlTD;
