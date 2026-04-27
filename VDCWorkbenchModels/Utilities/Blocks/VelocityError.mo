within VDCWorkbenchModels.Utilities.Blocks;
model VelocityError
  extends Modelica.Blocks.Icons.Block;

  VehicleInterfaces.Interfaces.ChassisBus chassisBus annotation (Placement(transformation(extent={{-16,-16},{16,16}},
        rotation=90,
        origin={-100,40})));
  Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{-16,-16},{16,16}},
        rotation=90,
        origin={-100,-40})));
  Modelica.Blocks.Interfaces.RealOutput longitudinal_velocity_error "Velocity error"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_velocity_in_path_direction
    annotation (Placement(transformation(extent={{100,-70},{120,-50}})));
  Modelica.Blocks.Interfaces.RealOutput vehicle_absolute_velocity
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
protected
  Modelica.Blocks.Interfaces.RealOutput yawAngle
    annotation (Placement(transformation(extent={{-40,70},{-20,90}})));
  Modelica.Blocks.Interfaces.RealOutput sideSlipAngle
    annotation (Placement(transformation(extent={{-40,50},{-20,70}})));
  Modelica.Blocks.Interfaces.RealOutput longitudinalVelocity
    annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
  Modelica.Blocks.Interfaces.RealOutput referenceVelocity
    annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  Modelica.Blocks.Interfaces.RealOutput referenceOrientation
    annotation (Placement(transformation(extent={{-40,-70},{-20,-50}})));

equation
  vehicle_absolute_velocity = longitudinalVelocity/cos(sideSlipAngle);
  vehicle_velocity_in_path_direction = cos(sideSlipAngle+yawAngle-referenceOrientation)*vehicle_absolute_velocity;
  longitudinal_velocity_error=vehicle_velocity_in_path_direction-referenceVelocity;

  connect(referenceVelocity, motionDemandBus.v_path) annotation (Line(points={{-30,-20},{-80,-20},{-80,-40},{-100,-40}},
                                      color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(referenceOrientation, motionDemandBus.psi_path) annotation (Line(
        points={{-30,-60},{-80,-60},{-80,-40},{-100,-40}},           color={0,0,
          127}), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(yawAngle, chassisBus.yawAngle) annotation (Line(points={{-30,80},{-72,80},{-72,42},{-100,42},{-100,40}},
                                   color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(sideSlipAngle, chassisBus.sideSlipAngle) annotation (Line(points={{-30,60},{-70,60},{-70,40},{-100,40}},
                                            color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  connect(longitudinalVelocity, chassisBus.longitudinalVelocity) annotation (
      Line(points={{-30,40},{-68,40},{-68,38},{-100,38},{-100,40}},
        color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{-2,2},{-2,5}},
      horizontalAlignment=TextAlignment.Right));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end VelocityError;
