within VDCWorkbenchModels.Utilities.Blocks;
model AssembleRLfeatureVec "Assemble RL feature vector from control bus signals"
  extends Modelica.Blocks.Icons.Block;

  parameter Real[nIn] featMax = {
    6.981317007977318,
    640.0,
    6.981317007977318,
    640.0,
    2.5,
    10.0,
    0.7853981633974483,
    0.1,
    50.0,
    1.117010721276371,
    150.0,
    50.0,
    10.0,
    4.0,
    0.17453292519943295,
    1.0};
    // [1]  baselineController_steeringWheelAngle [rad]
    // [2]  baselineController_torque [N.m]
    // [3]  combined_steeringWheelAngle [rad]
    // [4]  combined_torque [N.m]
    // [5]  lateralDisplacement_error [m]
    // [6]  longitudinalVelocity_error [m/s]
    // [7]  orientation_error [rad]
    // [8]  path_curvature [m-1]
    // [9]  path_velocity [m/s]
    // [10] residualRL_steeringWheelAngle [rad]
    // [11] residualRL_torque [N.m]
    // [12] vehicle_absoluteVelocity [m/s]
    // [13] vehicle_lateralAcceleration [m/s2]
    // [14] vehicle_longitudinalAcceleration [m/s2]
    // [15] vehicle_sideSlipAngle [rad]
    // [16] vehicle_yawRate [rad/s]

  parameter Real[nIn] featNominal = {
     0.6981317007977318,
     400.0,
     0.6981317007977318,
     400.0,
     0.5,
     3.0,
     0.08726646259971647,
     0.02,
     20.0,
     1.117010721276371,
     150.0,
     20.0,
     3.0,
     1.5,
     0.017453292519943295,
     0.15};
protected
  constant Integer nIn = 16 "Number of inputs";

public
  Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(transformation(
          extent={{-22,-22},{22,22}},
          rotation=90,
          origin={-100,0}),
        iconTransformation(
          extent={{-22,-22},{22,22}},
          rotation=90,
          origin={-100,0})));
  Modelica.Blocks.Interfaces.RealOutput featureVec[nIn]
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
//  Real[nIn] raw;
protected
  Modelica.Blocks.Interfaces.RealOutput[nIn] raw;

equation
/*
  raw[1]  = fmuOutputsBus.baselineController_steeringWheelAngle;
  raw[2]  = fmuOutputsBus.baselineController_torque;
  raw[3]  = fmuOutputsBus.combined_steeringWheelAngle;
  raw[4]  = fmuOutputsBus.combined_torque;
  raw[5]  = fmuOutputsBus.lateralDisplacement_error;
  raw[6]  = fmuOutputsBus.longitudinalVelocity_error;
  raw[7]  = fmuOutputsBus.orientation_error;
  raw[8]  = fmuOutputsBus.path_curvature;
  raw[9]  = fmuOutputsBus.path_velocity;
  raw[10] = fmuOutputsBus.residualRL_steeringWheelAngle;
  raw[11] = fmuOutputsBus.residualRL_torque;
  raw[12] = fmuOutputsBus.vehicle_absoluteVelocity;
  raw[13] = fmuOutputsBus.vehicle_lateralAcceleration;
  raw[14] = fmuOutputsBus.vehicle_longitudinalAcceleration;
  raw[15] = fmuOutputsBus.vehicle_sideSlipAngle;
  raw[16] = fmuOutputsBus.vehicle_yawRate;
*/

  // Clip to [min, max], then normalize by nominal
  for i in 1:nIn loop
    featureVec[i] = max(-featMax[i], min(featMax[i], raw[i])) / featNominal[i];
  end for;

  connect(raw[1], fmuOutputsBus.baselineController_steeringWheelAngle);
  connect(raw[2], fmuOutputsBus.baselineController_torque);
  connect(raw[3], fmuOutputsBus.combined_steeringWheelAngle);
  connect(raw[4], fmuOutputsBus.combined_torque);
  connect(raw[5], fmuOutputsBus.lateralDisplacement_error);
  connect(raw[6], fmuOutputsBus.longitudinalVelocity_error);
  connect(raw[7], fmuOutputsBus.orientation_error);
  connect(raw[8], fmuOutputsBus.path_curvature);
  connect(raw[9], fmuOutputsBus.path_velocity);
  connect(raw[10], fmuOutputsBus.residualRL_steeringWheelAngle);
  connect(raw[11], fmuOutputsBus.residualRL_torque);
  connect(raw[12], fmuOutputsBus.vehicle_absoluteVelocity);
  connect(raw[13], fmuOutputsBus.vehicle_lateralAcceleration);
  connect(raw[14], fmuOutputsBus.vehicle_longitudinalAcceleration);
  connect(raw[15], fmuOutputsBus.vehicle_sideSlipAngle);
  connect(raw[16], fmuOutputsBus.vehicle_yawRate);

  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Rectangle(
          extent={{-20,54},{-16,-54}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          lineThickness=0.5),
        Line(
          points={{-20,0},{-90,0}},
          color={255,204,51},
          thickness=0.5),
        Polygon(
          points={{-16,54},{2,44},{-16,34},{-16,54}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-16,32},{2,22},{-16,12},{-16,32}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-16,10},{2,0},{-16,-10},{-16,10}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-16,-12},{2,-22},{-16,-32},{-16,-12}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-16,-34},{2,-44},{-16,-54},{-16,-34}},
          lineColor={0,0,127},
          lineThickness=0.5,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Line(
          points={{2,-44},{100,0}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{2,-22},{100,0}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{2,0},{100,0}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{2,22},{98,0}},
          color={0,0,127},
          thickness=0.5),
        Line(
          points={{2,44},{100,0}},
          color={0,0,127},
          thickness=0.5)}),
    Diagram(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}})),
    Documentation(
      info="<html>
<p>
The output connector is the <strong>assembly</strong> of signals from the FMU&apos;s control bus.
Note, the number of signals is fixed and the signals are assigned to the output vector&apos;s index
as follows:
</p>
<ul>
  <li>[1]  .. baselineController_steeringWheelAngle / rad,</li>
  <li>[2]  .. baselineController_torque / N.m,</li>
  <li>[3]  .. combined_steeringWheelAngle / rad,</li>
  <li>[4]  .. combined_torque / N.m,</li>
  <li>[5]  .. lateralDisplacement_error / m,</li>
  <li>[6]  .. longitudinalVelocity_error / m.s<sup>-1</sup>,</li>
  <li>[7]  .. orientation_error / rad,</li>
  <li>[8]  .. path_curvature / m<sup>-1</sup>,</li>
  <li>[9]  .. path_velocity / m.s<sup>-1</sup>,</li>
  <li>[10] .. residualRL_steeringWheelAngle / rad,</li>
  <li>[11] .. residualRL_torque / N.m,</li>
  <li>[12] .. vehicle_absoluteVelocity / m.s<sup>-1</sup>,</li>
  <li>[13] .. vehicle_lateralAcceleration / m.s<sup>-2</sup>,</li>
  <li>[14] .. vehicle_longitudinalAcceleration / m.s<sup>-2</sup>,</li>
  <li>[15] .. vehicle_sideSlipAngle / rad,</li>
  <li>[16] .. vehicle_yawRate / rad.s<sup>-1</sup>,</li>
</ul>
</html>"));
end AssembleRLfeatureVec;
