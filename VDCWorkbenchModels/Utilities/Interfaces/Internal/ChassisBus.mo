within VDCWorkbenchModels.Utilities.Interfaces.Internal;
expandable connector ChassisBus
  "Do not use - Expandable connector defining signals for chassis bus"
  extends VehicleInterfaces.Interfaces.ChassisBus;

  Modelica.Units.SI.Angle yawAngle "Yaw angle of chassis";
  Modelica.Units.SI.Angle steeringWheelAngle "Steering wheel angle";
  Modelica.Units.SI.AngularVelocity yawRate "Yaw rate of chassis";
  Modelica.Units.SI.Angle sideSlipAngle "Side slip angle of chassis";
  Modelica.Units.SI.Distance travelledDistance "Travelled distance of chassis";
  Modelica.Units.SI.Acceleration longitudinalAcceleration
    "Longitudinal acceleration of chassis";
  Modelica.Units.SI.Velocity longitudinalVelocity "Longitudinal velocity of chassis";
  Modelica.Units.SI.Velocity longitudinalVelocity_start
    "Initial longitudinal velocity of chassis";

  //Real position_0[3] "Absolute position of chassis (x, y, phi) resolved in world frame";
  Modelica.Units.SI.Position position_x "Absolute position of chassis (x) resolved in world frame";
  Modelica.Units.SI.Position position_y "Absolute position of chassis (y) resolved in world frame";
  Modelica.Units.SI.Angle position_phi "Absolute position of chassis (phi) resolved in world frame";
  //Real velocity_0[3] "Absolute velocity of chassis (dx, dy, dphi) resolved in world frame";
  Modelica.Units.SI.Velocity velocity_dx "Absolute velocity of chassis (dx) resolved in world frame";
  Modelica.Units.SI.Velocity velocity_dy "Absolute velocity of chassis (dy) resolved in world frame";
  Modelica.Units.SI.AngularVelocity velocity_dphi "Absolute velocity of chassis (dphi) resolved in world frame";
  //Real acceleration_0[3] "Absolute acceleration of chassis (dx, dy, dphi) resolved in world frame";
  Modelica.Units.SI.Acceleration acceleration_ddx "Absolute acceleration of chassis (ddx) resolved in world frame";
  Modelica.Units.SI.Acceleration acceleration_ddy "Absolute acceleration of chassis (ddx) resolved in world frame";
  Modelica.Units.SI.AngularAcceleration acceleration_ddphi "Absolute acceleration of chassis (ddx) resolved in world frame";
  annotation (
    Documentation(info="<html>
<p>
An expandable connector that defines the minimum set of signals required on the <strong>chassisBus</strong>.
This connector shall <strong>not</strong> be used in models and is included here to enable
connection dialog (i.e. the GUI) for signal buses.
</p>
</html>"),
    Diagram(graphics={
        Text(
          extent={{-100,-40},{100,-80}},
          textColor={255,0,0},
          textString="Do not use!")}));
end ChassisBus;
