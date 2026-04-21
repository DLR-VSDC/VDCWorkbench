within VDCWorkbenchModels.Data;
record ROMOParametersResidualRL "ROMO's chassis parameters for residual reinforcement learning"
  extends VDCWorkbenchModels.Data.BaseRecords.Chassis(
    Torque_max_rearMotor=160,
    Torque_max_frontMotor=320,
    s={1,0},
    mu_S=1758/(m_vehicle*9.81/4),
    vSlide=21,
    mu_A=2671/(m_vehicle*9.81/4),
    vAdhesion=2.66,
    cr=50e3,
    cf=50e3,
    J_wheel=0.9,
    R0=0.27,
    area=1.95,
    c_W=0.55,
    steeringRatio=16,
    J_steer=5,
    trackWidth=1.45,
    wheelBase=2.398,
    Jz_vehicle=1130,
    m_body=690,
    m_vehicle=m_body,
    variantName="ROMO");

  annotation (
    Documentation(
      info="<html>
<p>
Set of common parameters of the <em>ROMO&apos;s</em> vehicle.
</p>
</html>"), Icon(graphics={
        Text(
          extent={{-140,-70},{140,-100}},
          textColor={0,0,0},
          textString="RL")}));
end ROMOParametersResidualRL;
