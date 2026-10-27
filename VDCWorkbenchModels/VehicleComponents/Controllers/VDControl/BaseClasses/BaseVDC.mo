within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.BaseClasses;
model BaseVDC "Interface definition for vehicle dynamics controllers"
  extends VehicleInterfaces.Icons.Controller;

  replaceable parameter Data.Tracks.Techlab2SBahnTIPI track constrainedby VDCWorkbenchModels.Data.Tracks.BaseTrack
    "Path to be evaluated"
    annotation(choicesAllMatching=true);

  parameter Modelica.Units.SI.Mass m=1000 "Mass of vehicle"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lf=1.2 "Distance of CoG to front axle"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length lr=1.2 "Distance of CoG to rear axle"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Length track_width=1.45 "Track width"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Inertia J=1000 "Yaw inertia"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Radius car_r = 0.3 "Tire radius"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Real cf(unit="N/rad")=30e3 "Front cornering stiffness"
    annotation(Dialog(group="Vehicle parameters"));
  parameter Real cr(unit="N/rad")=30e3 "Rear cornering stiffness"
    annotation(Dialog(group="Vehicle parameters"));

  parameter Real steer_gain=16 "Steering wheel gain" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Torque Torque_max_frontMotor=320
    "Maximum torque front motor" annotation(Dialog(group="Vehicle parameters"));
  parameter Modelica.Units.SI.Torque Torque_max_rearMotor=160
    "Maximum torque rear motor" annotation(Dialog(group="Vehicle parameters"));

  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus
    annotation (Placement(transformation(extent={{-20,-120},{20,-80}})));
  Modelica.Blocks.Interfaces.RealInput TV_ratio
    "Torque Vector ratio (=1 means all torque applied to right motor)"
    annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
  Modelica.Blocks.Interfaces.RealInput FrontRear_ratio
    "front-rear torque allocation ratio [0-1]" annotation (Placement(
        transformation(extent={{-140,-60},{-100,-20}})));
  Modelica.Blocks.Interfaces.RealInput v_scl "Down scale vector for velocity "
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
end BaseVDC;
