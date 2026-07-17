within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.Components;
model TorqueVectoring "Allocate driving torque to front central drive and left/right rear drives"
  extends Modelica.Blocks.Icons.Block;

  parameter Modelica.Units.SI.Length wheel_r =1 "Wheel radius [m]";
  parameter Modelica.Units.SI.Length track_width =1 "Track width [m]";
  parameter Modelica.Units.SI.Torque Torque_max_frontMotor=320 "Maximum torque front motor";
  parameter Modelica.Units.SI.Torque Torque_max_rearMotor=160 "Maximum torque rear motor";

  Modelica.Blocks.Interfaces.RealInput Torque(final unit="N.m") "Total torque applied to motors"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput alpha_TV
    "Torque vectoring ratio (=1 all torque applied to right wheel)"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.RealInput alpha_FrontRear "Front/rear allocation ratio (=1: all torque applied to front drive)"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput torque_dem_front(final unit="N.m")
    "Torque demand to electric machine (Front)"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  Modelica.Blocks.Interfaces.RealOutput torque_dem_rearLeft(final unit="N.m") "Torque demand to electric machine rear left"
    annotation (Placement(transformation(extent={{100,-30},{120,-10}})));
  Modelica.Blocks.Interfaces.RealOutput torque_dem_rearRight(final unit="N.m") "Torque demand to electric machine rear right"
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Interfaces.RealOutput Mz_TorqueVectoring(final unit="N.m")
    "Yaw moment due to torque vectoring" annotation (Placement(
        transformation(
        extent={{100,10},{120,30}})));
  Modelica.Blocks.Nonlinear.Limiter alpha_lim(final uMax=1, final uMin=0)
    annotation (Placement(transformation(extent={{-90,-70},{-70,-50}})));
  Modelica.Blocks.Math.Product product1
    annotation (Placement(transformation(extent={{-30,-50},{-10,-30}})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{10,-30},{30,-10}})));
  Modelica.Blocks.Math.Product product2
    annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
  Modelica.Blocks.Nonlinear.Limiter alpha_lim1(final uMax=1, final uMin=0)
    annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
  Modelica.Blocks.Math.Add add1(k1=-1, k2=+1)
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
  Modelica.Blocks.Nonlinear.Limiter alpha_lim2(
    uMax=Torque_max_frontMotor,
    uMin=-Torque_max_frontMotor)
    annotation (Placement(transformation(extent={{60,50},{80,70}})));
  Modelica.Blocks.Nonlinear.Limiter tauMotorL_lim(uMax=Torque_max_rearMotor, uMin=-Torque_max_rearMotor) annotation (Placement(transformation(extent={{40,-30},{60,-10}})));
  Modelica.Blocks.Nonlinear.Limiter tauMotorR_lim(uMax=Torque_max_rearMotor, uMin=-Torque_max_rearMotor) annotation (Placement(transformation(extent={{40,-70},{60,-50}})));
  Modelica.Blocks.Math.Add tauRearDifference(k2=-1) annotation (Placement(transformation(extent={{72,-32},{88,-48}})));
  Modelica.Blocks.Math.Gain gain(k=track_width/2/wheel_r) annotation (Placement(transformation(extent={{60,10},{80,30}})));
equation
  connect(alpha_TV, alpha_lim.u)
    annotation (Line(points={{-120,-60},{-92,-60}}, color={0,0,127}));
  connect(alpha_lim.y, product1.u2)
    annotation (Line(points={{-69,-60},{-40,-60},{-40,-46},{-32,-46}},
        color={0,0,127}));
  connect(product1.y, add.u2) annotation (Line(points={{-9,-40},{0,-40},{0,-26},{8,-26}},
        color={0,0,127}));
  connect(Torque, product2.u1) annotation (Line(points={{-120,60},{-80,60},{-80,66},{-62,66}},
        color={0,0,127}));
  connect(product1.u1, add.u1) annotation (Line(points={{-32,-34},{-40,-34},{-40,-14},{8,-14}},
        color={0,0,127}));
  connect(alpha_FrontRear, alpha_lim1.u)
    annotation (Line(points={{-120,0},{-92,0}}, color={0,0,127}));
  connect(alpha_lim1.y, product2.u2) annotation (Line(points={{-69,0},{-66,0},{-66,54},{-62,54}},
        color={0,0,127}));
  connect(product2.y, add1.u1) annotation (Line(points={{-39,60},{-36,60},{-36,36},{-32,36}},
        color={0,0,127}));
  connect(Torque, add1.u2) annotation (Line(points={{-120,60},{-80,60},{-80,24},{-32,24}},
        color={0,0,127}));
  connect(add1.y, add.u1) annotation (Line(points={{-9,30},{0,30},{0,-14},{8,-14}},
        color={0,0,127}));
  connect(product2.y, alpha_lim2.u)
    annotation (Line(points={{-39,60},{58,60}}, color={0,0,127}));
  connect(alpha_lim2.y, torque_dem_front)
    annotation (Line(points={{81,60},{110,60}}, color={0,0,127}));
  connect(add.y, tauMotorL_lim.u) annotation (Line(points={{31,-20},{38,-20}}, color={0,0,127}));
  connect(tauMotorL_lim.y, torque_dem_rearLeft) annotation (Line(points={{61,-20},{110,-20}}, color={0,0,127}));
  connect(product1.y, tauMotorR_lim.u) annotation (Line(points={{-9,-40},{0,-40},{0,-60},{38,-60}}, color={0,0,127}));
  connect(tauMotorR_lim.y, torque_dem_rearRight) annotation (Line(points={{61,-60},{110,-60}}, color={0,0,127}));
  connect(tauRearDifference.y, gain.u) annotation (Line(points={{88.8,-40},{92,-40},{92,0},{50,0},{50,20},{58,20}},
        color={0,0,127}));
  connect(tauMotorR_lim.y, tauRearDifference.u1) annotation (Line(points={{61,-60},{66,-60},{66,-44.8},{70.4,-44.8}}, color={0,0,127}));
  connect(tauMotorL_lim.y, tauRearDifference.u2) annotation (Line(points={{61,-20},{66,-20},{66,-35.2},{70.4,-35.2}}, color={0,0,127}));
  connect(gain.y, Mz_TorqueVectoring) annotation (Line(points={{81,20},{110,20}}, color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Ellipse(
          extent={{-90,90},{90,-90}},
          lineColor={28,108,200},
          lineThickness=1),
          Text(
          extent={{-90,10},{-10,-20}},
          textColor={0,0,0},
          textString="TV"),
        Rectangle(
          extent={{-8,34},{8,-34}},
          lineColor={0,0,0},
          radius=10,
          origin={0,50},
          rotation=50,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-8,34},{8,-34}},
          lineColor={0,0,0},
          radius=10,
          origin={0,-52},
          rotation=-30,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-6,50},{6,-50}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Line(
          points={{34,-66},{64,-64},{54,-88}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-54,92},{-64,64},{-34,68}},
          color={238,46,47},
          thickness=0.5)}),
    Documentation(info="<html>
<p>
The required total torque&nbsp;<var>&tau;</var>, provided by the input <code>torque</code>,
is divided between the front and rear axle by the variable front/rear ratio
&alpha;<sub>AD</sub>&nbsp;&isin;&nbsp;[0, 1]
(input <code>alpha_FrontRear</code>) like
</p>
<blockquote>
<var>&tau;</var><sub>F</sub> = <var>&tau;</var> * &alpha;<sub>AD</sub>, <br>
<var>&tau;</var><sub>R</sub> = <var>&tau;</var> * (1 - &alpha;<sub>AD</sub>),
</blockquote>
<p>
whereby the index&nbsp;F stands for &quot;front&quot; and&nbsp;R for &quot;rear&quot;.
Afterwards, the rear axle torque is allocated into left and right motor torques using
a&nbsp;normalized torque vector ratio &alpha;<sub>TV</sub>&nbsp;&isin;&nbsp;[0, 1],
which is the input <code>alpha_TV</code>,
</p>
<blockquote>
<var>&tau;</var><sub>RR</sub> = <var>&tau;</var><sub>R</sub> * &alpha;<sub>TV</sub>, <br>
<var>&tau;</var><sub>RL</sub> = <var>&tau;</var><sub>R</sub> * (1 - &alpha;<sub>TV</sub>),
</blockquote>
<p>
with the index&nbsp;RR for &quot;rear right&quot; and&nbsp;RL for &quot;rear left&quot;.
When &alpha;<sub>TV</sub>&nbsp;=&nbsp;0.5, both motors receive the same torque;
&alpha;<sub>TV</sub>&nbsp;=&nbsp;1 allocates all the torque solely to the right motor
and &alpha;<sub>TV</sub>&nbsp;=&nbsp;0 solely to the left motor.
</p>
<p>
Additionally, all torques are subject to constraints:
</p>
<blockquote>
-<var>&tau;</var><sub>max,<var>i</var></sub> &le; <var>&tau;<sub>i</sub></var> &le; <var>&tau;</var><sub>max,<var>i</var></sub> ,
</blockquote>
<p>
where <var>i</var>&nbsp;&isin;&nbsp;{F, RR, RL} is the index of the corresponding machine and
<var>&tau;<sub>max,i</sub></var> its the maximum allowed torque.
</p>
</html>"));
end TorqueVectoring;
