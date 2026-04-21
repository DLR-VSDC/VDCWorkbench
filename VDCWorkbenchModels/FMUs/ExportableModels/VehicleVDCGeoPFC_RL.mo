within VDCWorkbenchModels.FMUs.ExportableModels;
model VehicleVDCGeoPFC_RL
  extends VehicleArchitectures.VDCWorkbench2025(
    redeclare VDCWorkbenchModels.Data.ROMOParametersResidualRL data,
    v_Start=max(path_v_init + initial_velocity_offset.value, 1.0),
    redeclare VehicleComponents.Controllers.VDControl.VDCWorkbenchControl_RL controller(
      v_scale=v_scl.value,
      tIPI_bus(
        FilePath=ModelicaServices.ExternalReferences.loadResource("modelica://VDCWorkbenchModels/Resources/Maps/") + pathName,
        maxArcLength=maxArcLength,
        Path(tableName="path"))),
    vehicle(
      carBody(
        body(
          r(start={x_start,y_start}),
          v(start={cos(yaw_start)*vehicle.carBody.v_long,sin(yaw_start)*vehicle.carBody.v_long}),
          phi(start=yaw_start, fixed=true)))));

  import Modelica.Units.Conversions.from_deg;

  parameter String pathName = "Racetrack.mat" "File name of the path definition #RL";
  parameter Modelica.Units.SI.Distance maxArcLength = 155.0 "Max arc length of the path #RL" annotation(Evaluate=false);
  parameter Modelica.Units.SI.Position path_x_init = 0.0 "Initial x position of CoG #RL";
  parameter Modelica.Units.SI.Position path_y_init = 0.0 "Initial x position of CoG #RL";
  parameter Modelica.Units.SI.Angle path_yaw_init = from_deg(0) "Initial yaw angle of CoG #RL";
  parameter Modelica.Units.SI.Velocity path_v_init = 9.5916 "Initial absolute velocity of CoG #RL";

  parameter Credibility.Scalar v_scl(
    value=1.0,
    traceability(
      source = Credibility.Types.SourceType.Estimated,
      info = "Scaling factor for the reference path velocity to vary velocity profiles during training. #RL"),
    redeclare Credibility.Types.Uniform uncertainty(
      unitValue="",
      lower=0.5,
      upper=1.0,
      source=Credibility.Types.SourceType.Provided,
      reference="Uniform scaling is defined by the user to equally emphasize velocity profiles within the scaling range."));

  parameter Credibility.Scalar initial_lateral_offset(value=0.0,
    traceability(
      source = Credibility.Types.SourceType.Estimated,
      info = "Initial lateral vehicle offset to randomize initialization during training. #RL"),
    redeclare Credibility.Types.Uniform uncertainty(
      unitValue="m",
      lower=-1.5,
      upper=1.5,
      source=Credibility.Types.SourceType.Provided,
      reference="Uniform scaling is defined by the user."));

  parameter Credibility.Scalar initial_yaw_offset(value=0.0,
    traceability(
      source = Credibility.Types.SourceType.Estimated,
      info = "Initial vehicle yaw offset to randomize initialization during training. #RL"),
    redeclare Credibility.Types.Uniform uncertainty(
      unitValue="rad",
      lower=-from_deg(10),
      upper=from_deg(10),
      source=Credibility.Types.SourceType.Provided,
      reference="Uniform scaling is defined by the user."));

  parameter Credibility.Scalar initial_velocity_offset(value=0.0,
    traceability(
      source = Credibility.Types.SourceType.Estimated,
      info = "Initial vehicle velocity offset to randomize initialization during training. #RL"),
    redeclare Credibility.Types.Uniform uncertainty(
      unitValue="m/s",
      lower=-5,
      upper=5,
      source=Credibility.Types.SourceType.Provided,
      reference="Uniform scaling is defined by the user."));

  Modelica.Blocks.Interfaces.RealInput residualRL_steeringWheelAngle_in(
    unit="rad",
    min=-from_deg(4)*data.steeringRatio,
    max=from_deg(4)*data.steeringRatio,
    nominal=from_deg(4)*data.steeringRatio)
    "Steering wheel angle commanded by the DRL controller, which is added to the steeringWheelAngle of the baseline controller"
    annotation (Placement(transformation(extent={{-142,38},{-102,78}})));
  Modelica.Blocks.Interfaces.RealInput residualRL_torque_in(
    unit="N.m",
    min=-150,
    max=150,
    nominal=150)
    "Torque commanded by the DRL controller, which is added to the torque of the baseliJa. ne controller"
    annotation (Placement(transformation(extent={{-140,-10},{-100,30}})));
  Utilities.Blocks.MapFmuOutputsBusSignals calcRLoutputs(
    steeringRatio=data.steeringRatio) annotation (Placement(transformation(extent={{60,-30},{80,-10}})));

protected
  parameter Modelica.Units.SI.Position x_start = path_x_init - sin(path_yaw_init)*initial_lateral_offset.value "Start value for the x position of the vehicle, which is assembled out of the initial value and the randomized  initial_lateral_offset";
  parameter Modelica.Units.SI.Position y_start = path_y_init + cos(path_yaw_init)*initial_lateral_offset.value "Start value for the y position of the vehicle, which is assembled out of the initial value and the randomized  initial_lateral_offset";
  parameter Modelica.Units.SI.Angle yaw_start = path_yaw_init + initial_yaw_offset.value "Start value for the yaw angle of the vehicle, which is assembled out of the initial yaw value and the randomized initial_yaw_offset";

public
  Utilities.Interfaces.FmuOutputsBus fmuOutputsBus annotation (Placement(
        transformation(
          extent={{-16,-16},{16,16}},
          rotation=-90,
          origin={100,-20})));
equation
  connect(controller.addRL_FrontSteering_in, residualRL_steeringWheelAngle_in)
    annotation (Line(points={{-42,58},{-42,58},{-122,58}}, color={0,0,127}));
  connect(controller.addRL_TotalTorque_in, residualRL_torque_in)  annotation (
      Line(points={{-42,54},{-90,54},{-90,10},{-120,10}}, color={0,0,127}));
  connect(calcRLoutputs.controlBus, controlBus) annotation (
      Line(
        points={{60,-20},{52,-20},{52,0},{100,0}},
        color={255,204,51},
        thickness=0.5),
      Text(
        string="%second",
        index=1,
        extent={{-2,2},{-2,5}},
        horizontalAlignment=TextAlignment.Right));
  connect(calcRLoutputs.fmuOutputsBus, fmuOutputsBus) annotation (Line(
      points={{80,-20},{100,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(calcRLoutputs.residualRL_steeringWheelAngle_in, residualRL_steeringWheelAngle_in) annotation (Line(points={{58.6667,-24},{40,-24},{40,66},{-60,66},{-60,58},{-122,58}},
        color={0,0,127}));
  connect(calcRLoutputs.residualRL_torque_in, residualRL_torque_in)
    annotation (Line(points={{58.6667,-28.2},{42,-28.2},{42,-28},{36,-28},{36,20},{-90,20},{-90,10},{-120,10}},
        color={0,0,127}));
  annotation (
    Diagram(coordinateSystem(extent={{-100,-100},{100,100}})),
    Icon(coordinateSystem(extent={{-100,-100},{100,100}})));
end VehicleVDCGeoPFC_RL;
