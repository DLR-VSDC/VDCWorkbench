within VDCWorkbenchModels.Data;
record MiniAFMChassis "MiniAFM's chassis parameters"
  extends BaseRecords.Chassis(
    m_body=0.9 * (m_vehicle - 4*0.25),
    variantName="MiniAFM",
    m_vehicle=7.151,
    trackWidth=0.265,
    wheelBase=0.361,
    Jz_vehicle=1/12 * m_vehicle * (trackWidth^2 + wheelBase^2),
    s={1,0},
    c_W = 0.0,
    area = 0.1,
    R0=0.0525,
    vAdhesion=1.0,
    vSlide=1.5,
    mu_A= 0.8,
    mu_S= 0.7,
    J_wheel=1/2 * 0.25 * R0^2,
    steeringRatio = 0.9,
    J_steer=0);

  annotation (
    Documentation(
      info="<html>
<p>
Set of common parameters of the <em>MiniAFM&apos;s</em> vehicle.
</p>
</html>"));
end MiniAFMChassis;
