within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased;
model PredictiveStanleyControlTD "Time-discrete predictive Stanley lateral control law"
  extends BaseClasses.BaseStanley;
  parameter Real k = 5 "Stanley gain";
  parameter Real v_eps = 0.1  "Small speed to avoid div by zero";
  parameter Real k_d_yaw = 0.14;
  parameter Real k_d_steer = 0.0;

  parameter Real deltaMax = 0.3 "Steering saturation [rad]";

  parameter Real K_vctr = 0.5 "P gain of velocity controller";
  parameter Modelica.Units.SI.Torque vctr_TorqueMax = 0.3;

  parameter Modelica.Units.SI.Time Ts = 0.05 "Controller sample time";

  parameter Real m = 7.151 "Vehicle mass [kg]";
  parameter Real lf = 0.1805;
  parameter Real lr = 0.1805;
  parameter Real C_Tire = 150 "Tire stiffnes for slip angle compensation";

  parameter Boolean use_prediction = true;
  parameter Real weights[5] = {0.4, 0.2, 0.2, 0.1, 0.1};
  parameter Integer N = 4;
  parameter Real dt = Ts;

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
  Real e_psi;
  Real delta_yaw;
  Real delta_steer;
  Real dpsi;
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

    // Stanley Controll Law
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
