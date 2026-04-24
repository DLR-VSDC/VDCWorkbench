within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.GeoPFC;
model MotionDemand
  //Implementation based on https://elib.dlr.de/105041/1/IV2016_PeterRitzer.pdf based on Python Code from Kenan Ahmic c DLR 2025

  parameter Real lambda_eLat = 10 "Lateral deviation";
  parameter Real lambda_del_psi = 20 "Yaw deviation";
  parameter Real e_y_ref = 0 "Eccentric parameter to distinguish road side";
protected
  parameter Real del_psi_ref = 0 "Yaw rate offset - only for all wheel steering";
  parameter Real d_del_psi_ref_dq = 0 "Yaw acceleration offset - only for all wheel steering";

public
  Real ePy,ePx,b,a_raw,a_max,a,radix,zeta,zeta_1,zeta_2,del_psi,beta_star,kappaICR_star;

protected
  VDCWorkbenchModels.Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-20,-10},{0,10}})));
public
  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={100,0})));
  Modelica.Blocks.Sources.RealExpression kappaICR(y=kappaICR_star)
    "Desired curvature at instantious center of rotation of PFC"
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
protected
  Modelica.Blocks.Sources.RealExpression beta(y=beta_star)
    "Desired side slip angle of geo pfc" annotation (Placement(transformation(
          extent={{-60,70},{-40,90}}), iconTransformation(extent={{-52,62},{-30,
            84}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{70,-10},{90,10}})));
  Modelica.Blocks.Interfaces.RealOutput xveh "Measured vehicle position, x" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,50})));
  Modelica.Blocks.Interfaces.RealOutput yveh "Measured vehicle position, y" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,30})));
  Modelica.Blocks.Interfaces.RealOutput psiveh "Measured vehicle position, psi" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,10})));
  Modelica.Blocks.Interfaces.RealOutput vveh "Absolute vehicle speed" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-10})));
  Modelica.Blocks.Interfaces.RealOutput betaveh "Side slip angle" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-30})));
  Modelica.Blocks.Interfaces.RealOutput psidotveh "Yaw rate" annotation (Placement(transformation(extent={{-10,-10},{10,10}}, rotation=180, origin={40,-50})));
  Modelica.Blocks.Interfaces.RealOutput x_path
    "Position x of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,32})));
  Modelica.Blocks.Interfaces.RealOutput y_path
    "Position y of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,12})));
  Modelica.Blocks.Interfaces.RealOutput psi_path
    "Position psi of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,-8})));
  Modelica.Blocks.Interfaces.RealOutput v_path
    "Velocity of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,-28})));
  Modelica.Blocks.Interfaces.RealOutput kappa_path
    "Curvature of path at current arc length value" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-50,-48})));
equation

  //def calculate_motion_demand(self, path_reference_point, vehicle_state):
  //lateral displacement calculation to path Eq. (14)
  ePy = -sin(psi_path)*(x_path-xveh) + cos(psi_path)*(y_path-yveh);
  //longitudinal displacement calculation
  ePx = cos(psi_path)*(x_path-xveh) + sin(psi_path)*(y_path-yveh);
  //orientation displacement calculation
  del_psi = atan2(sin(psi_path - psiveh), cos(psi_path - psiveh));

  //coefficient calculation for Eq. 22

  // Eq. (24) in https://elib.dlr.de/105041/1/IV2016_PeterRitzer.pdf
  b = kappa_path*ePx/(ePy*kappa_path + 1);
  // Eq. (23) in https://elib.dlr.de/105041/1/IV2016_PeterRitzer.pdf
  a_raw = (e_y_ref - ePy)/lambda_eLat;
  a_max = sqrt(b^2 + 1);
  a = max(min(a_raw, a_max), -a_max);

  // the two solutions of Eq. (22) in https://elib.dlr.de/105041/1/IV2016_PeterRitzer.pdf
  radix = (max(-a^2 + b^2 + 1, 0))^(0.5); // prevent sqrt of a negative number
  zeta_1 = atan2(-b * (-a * b + radix) / (b^2 + 1) - a, (-a * b + radix) / (b^2 + 1));
  zeta_2 = atan2(b * (a * b + radix) /( b^2 + 1) - a,-(a*b + radix)/(b^2 +1));

  if abs(zeta_1) < Modelica.Constants.pi/2 then
     zeta = zeta_1;
  else
     zeta = zeta_2;
  end if;

  //Setpoints for control allocation
  //Solution of Eq. 26
  beta_star = zeta + del_psi;
  //Solution of Eq. 27
  kappaICR_star = (del_psi - del_psi_ref)/lambda_del_psi - d_del_psi_ref_dq + (kappa_path * cos(beta_star - del_psi))/(ePy*kappa_path + 1);
  connect(xveh, chassisBus.position_x) annotation (Line(points={{40,50},{80,50},{80,0}}, color={0,0,127}),
      Text(string="%second", index=1, extent={{12,3},{12,3}}, horizontalAlignment=TextAlignment.Left));
  connect(yveh, chassisBus.position_y) annotation (Line(points={{40,30},{80,30},{80,0}}, color={0,0,127}),
      Text(string="%second", index=1, extent={{12,3},{12,3}}, horizontalAlignment=TextAlignment.Left));
  connect(psiveh, chassisBus.yawAngle) annotation (Line(points={{40,10},{80,10},{80,0}}, color={0,0,127}),
      Text(string="%second", index=1, extent={{12,3},{12,3}}, horizontalAlignment=TextAlignment.Left));
  connect(betaveh, chassisBus.sideSlipAngle) annotation (Line(points={{40,-30},{80,-30},{80,0}}, color={0,0,127}),
      Text(string="%second", index=1, extent={{12,3},{12,3}}, horizontalAlignment=TextAlignment.Left));
  connect(psidotveh, chassisBus.yawRate) annotation (Line(points={{40,-50},{80,-50},{80,0}}, color={0,0,127}),
      Text(string="%second", index=1, extent={{12,3},{12,3}}, horizontalAlignment=TextAlignment.Left));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{80,0},{90,0},{90,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
      points={{-10,0},{10,0},{10,70},{100.1,70},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(x_path, motionDemandBus.x_path)
    annotation (Line(points={{-50,32},{-12,32},{-12,0},{-10,0}}, color={0,0,127}));
  connect(y_path, motionDemandBus.y_path) annotation (Line(points={{-50,12},{-14,12},{-14,0},{-10,0}}, color={0,0,127}));
  connect(psi_path, motionDemandBus.psi_path) annotation (Line(points={{-50,-8},{-12,-8},{-12,0},{-10,0}}, color={0,0,127}));
  connect(v_path, motionDemandBus.v_path)
    annotation (Line(points={{-50,-28},{-10,-28},{-10,0}}, color={0,0,127}));
  connect(kappa_path, motionDemandBus.kappa_path) annotation (Line(points={{-50,-48},{-8,-48},{-8,0},{-10,0}}, color={0,0,127}));
  connect(beta.y, motionDemandBus.beta_geo)
    annotation (Line(points={{-39,80},{-8,80},{-8,0},{-10,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(kappaICR.y, motionDemandBus.kappaICR_geo) annotation (Line(points={{-39,60},{-10,60},{-10,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(vveh, chassisBus.longitudinalVelocity) annotation (Line(points={{40,-10},{80,-10},{80,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false),
      graphics={
        Polygon(
          points={{-100,-100},{-56,100},{60,100},{100,-100},{-100,-100}},
          lineColor={28,108,200},
          fillColor={102,44,145},
          fillPattern=FillPattern.Forward),
        Text(
          extent={{-100,60},{100,0}},
          textColor={255,255,255},
          textString="Motion
Demand")}));
end MotionDemand;
