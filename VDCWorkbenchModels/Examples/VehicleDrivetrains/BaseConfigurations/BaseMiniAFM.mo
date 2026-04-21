within VDCWorkbenchModels.Examples.VehicleDrivetrains.BaseConfigurations;
model BaseMiniAFM "Basic architecture of the miniAFM"
  extends VehicleArchitectures.BaseArchitecture(
    break data,
    vehicle(
      trackWidth=data.trackWidth,
      wheelBase=data.wheelBase,
      Jz=data.Jz_vehicle,
      m=data.m_vehicle,
      J_steer=data.J_steer,
      s=data.s,
      R0=data.R0,
      vAdhesion=data.vAdhesion,
      vSlide=data.vSlide,
      mu_A=data.mu_A,
      mu_S=data.mu_S,
      J_wheel=data.J_wheel,
      N=data.m_vehicle*9.81/4,
      c_W=data.c_W,
      area=data.area,
      r=data.s,
      axleFront(steeringGear(ratio=0.9))),
    break chassisControlBus);
  extends Modelica.Icons.Example;

  Data.MiniAFMChassis data
    annotation (Placement(transformation(extent={{40,80},{60,100}})));
end BaseMiniAFM;
