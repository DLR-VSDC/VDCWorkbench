within VDCWorkbenchModels.VehicleComponents.Chassis.Components;
model CarBody "Planar rigid body"
  parameter Modelica.Units.SI.Length wheelBase = 2.398 "Wheelbase";
  parameter Modelica.Units.SI.Inertia Jz=1 "Yaw inertia";
  parameter Modelica.Units.SI.Mass m=1 "Mass";
  parameter Modelica.Units.SI.Velocity v_long = 0 "Initial velocity in longitudinal direction";

  parameter Boolean animate=true "= true, if animation shall be enabled" annotation (Dialog(group="Animation"));

  PlanarMechanics.Parts.Body body(
    animate=animate,
    stateSelect=StateSelect.prefer,
    m=m,
    I=Jz,
    v(start={v_long,0})) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=180,
        origin={-30,0})));
  PlanarMechanics.Parts.FixedTranslation frontAxle(
    animate=animate,
    r={wheelBase/2,0}) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, 70})));
  PlanarMechanics.Parts.FixedTranslation rearAxle(
    animate=animate,
    r={wheelBase/2,0}) annotation(
      Placement(transformation(extent = {{-10, -10}, {10, 10}}, rotation = 90, origin = {0, -70})));
  PlanarMechanics.Interfaces.Frame_a frameAxleFront "Reference frame of front axle" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={0,100})));
  PlanarMechanics.Interfaces.Frame_b frameAxleRear "Reference frame of rear axle" annotation (Placement(transformation(
        extent={{-16,-16},{16,16}},
        rotation=90,
        origin={0,-100})));
  Utilities.Interfaces.ControlBus controlBus annotation(
    Placement(transformation(extent = {{-20, -20}, {20, 20}}, rotation = 90, origin = {100, 60})));
  VDCWorkbenchModels.Utilities.VehicleDynamicsSensor vehDynSensor annotation(
    Placement(transformation(extent = {{20, -10}, {40, 10}})));
equation
  connect(rearAxle.frame_a, frameAxleRear) annotation (Line(
      points={{-6.66134e-16,-80},{0,-80},{0,-100}},
      color={95,95,95},
      thickness=0.5));
  connect(frontAxle.frame_b, frameAxleFront) annotation (Line(
      points={{0,80},{0,100}},
      color={95,95,95},
      thickness=0.5));
  connect(frontAxle.frame_a, body.frame_a) annotation(
    Line(points = {{0, 60}, {0, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rearAxle.frame_b, body.frame_a) annotation(
    Line(points = {{6.66134e-16, -60}, {6.66134e-16, 0}, {-20, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(vehDynSensor.frame_CoG, body.frame_a) annotation(
    Line(points={{20,0},{-20,0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(vehDynSensor.controlBus, controlBus) annotation(
    Line(points = {{40, 0}, {80, 0}, {80, 60}, {100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  annotation(
    Icon(coordinateSystem(preserveAspectRatio = false, extent = {{-100, -100}, {100, 100}}),
      graphics = {
        Line(
          points = {{0, 100}, {0, 90}},
          color = {135, 135, 135}),
        Line(
          points = {{0, 50}, {0, -100}},
          color = {135, 135, 135}),
        Polygon(
          points = {{58, 26}, {10, 36}, {-30, 20}, {-40, -8}, {-72, -30}, {-76, -38}, {-76, -58}, {-28, -68}, {86, -10}, {86, 8}, {84, 14}, {58, 26}},
          lineColor = {135, 135, 135},
          smooth = Smooth.None,
          fillColor = {170, 213, 255},
          fillPattern = FillPattern.Solid),
        Line(
          points = {{8, -18}, {-40, -8}},
          color = {135, 135, 135},
          smooth = Smooth.None),
        Line(
          points = {{58, 26}, {18, 10}, {8, -18}, {-24, -40}, {-28, -48}, {-28, -68}},
          color = {135, 135, 135}, smooth = Smooth.None),
        Line(
          points = {{18, 10}, {-30, 20}},
          color = {135, 135, 135}, smooth = Smooth.None),
        Line(
          points = {{-24, -40}, {-72, -30}},
          color = {135, 135, 135}, smooth = Smooth.None),
        Ellipse(
          extent = {{58, -4}, {74, -38}},
          lineColor = {135, 135, 135},
          fillColor = {135, 135, 135},
          fillPattern = FillPattern.Solid,
          startAngle=168,
          endAngle=345,
          closure=EllipseClosure.Radial),
        Ellipse(
          extent = {{-12, -40}, {4, -74}},
          lineColor = {135, 135, 135},
          fillColor = {135, 135, 135},
          fillPattern = FillPattern.Solid,
          startAngle=168,
          endAngle=345,
          closure=EllipseClosure.Radial),
        Text(
          extent = {{-150, 90}, {150, 50}},
          textColor = {0, 0, 255},
          textString = "%name")}),
    Diagram,
    Documentation(info="<html>
<p>
Planar vehicle&apos;s body connecting steerable front axle and rear axle to
vehicle&apos;s mass. Basic measurements of vehicle&apos;s planar dynamic are placed
on the chassis bus.
</p>
</html>"));
end CarBody;
