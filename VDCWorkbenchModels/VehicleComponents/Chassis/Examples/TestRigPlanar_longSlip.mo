within VDCWorkbenchModels.VehicleComponents.Chassis.Examples;
model TestRigPlanar_longSlip "Simple test for single and dual wheels bouncing on road"
  extends Modelica.Icons.Example;
  import Modelica.Constants.pi;

  parameter Modelica.Units.SI.Force load = 2000
    "Load applied in the wheel in z-direction";
  parameter Modelica.Units.SI.Velocity speed = 80/3.6
    "Longitudinal speed of the wheel : 80km/h in m/s";
  parameter Modelica.Units.SI.Distance s_start_Z=wheelSingle.radius "Initial value of the height of the wheel center compared to the road";

  PlanarMechanics.VehicleComponents.Wheels.DryFrictionWheelJoint wheelSingle(
    radius=0.27,
    r={1,0},
    N=1046/4*9.81,
    vAdhesion=2.66,
    vSlide=21,
    mu_A=2671/wheelSingle.N,
    mu_S=1758/wheelSingle.N,
    w_roll(start=speed/wheelSingle.radius),
    v_long(start=speed)) annotation (Placement(transformation(extent={{0,-30},{20,-10}})));
  inner PlanarMechanics.PlanarWorld planarWorld(constantGravity={0,0})
    annotation (Placement(transformation(extent={{10,10},{-10,-10}},
        rotation=180,
        origin={-90,-20})));

  PlanarMechanics.Joints.Prismatic prismatic_x(
    useFlange=true,
    r={1,0},
    boxWidth=0.01,
    s(start=0, fixed=true)) annotation (Placement(transformation(extent={{-40,-10},{-20,-30}})));
  Modelica.Mechanics.Translational.Sources.ConstantSpeed constantSpeed(
    v_fixed(displayUnit="km/h") = speed)
    annotation (Placement(transformation(extent={{-10,10},{-30,30}})));
  Modelica.Mechanics.Rotational.Sources.Speed speedWheel annotation (Placement(transformation(extent={{60,-30},{40,-10}})));
  Modelica.Blocks.Sources.Sine sine(
    f=0.00025,
    phase=3*pi/2,
    amplitude=2*speed/wheelSingle.radius,
    offset=2*speed/wheelSingle.radius) annotation (Placement(transformation(extent={{100,-30},{80,-10}})));
  PlanarMechanics.Parts.Fixed fixed annotation (Placement(transformation(extent={{-50,-30},{-70,-10}})));
equation
  connect(sine.y, speedWheel.w_ref) annotation (Line(
      points={{79,-20},{62,-20}},
      color={0,0,127},
      smooth=Smooth.None));
  connect(constantSpeed.flange, prismatic_x.axis) annotation (Line(points={{-30,20},{-30,-10}}, color={0,127,0}));
  connect(prismatic_x.frame_b, wheelSingle.frame_a) annotation (Line(
      points={{-20,-20},{6,-20}},
      color={95,95,95},
      thickness=0.5));
  connect(wheelSingle.flange_a, speedWheel.flange) annotation (Line(points={{20,-20},{40,-20}}, color={0,0,0}));
  connect(prismatic_x.frame_a, fixed.frame) annotation (Line(
      points={{-40,-20},{-50,-20}},
      color={95,95,95},
      thickness=0.5));
  annotation (
    Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
      graphics={
        Text(
          extent={{20,42},{100,20}},
          textColor={28,108,200},
          textString="Parameters of ROMO tire"),
        Line(
          points={{26,24},{14,-6}},
          color={28,108,200},
          arrow={Arrow.None,Arrow.Open})}),
    Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{100,100}})),
    Documentation(
      info="<html>
<p>
Test rig of a wheelSingle model to obtain specific characteristics with a continuous variation of the longitudinal slip.
</p>
</html>"),
    experiment(
      StopTime=1000,
      __Dymola_NumberOfIntervals=10000,
      __Dymola_Algorithm="Dassl"));
end TestRigPlanar_longSlip;
