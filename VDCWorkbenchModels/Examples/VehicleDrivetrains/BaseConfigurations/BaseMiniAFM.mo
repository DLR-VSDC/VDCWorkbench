within VDCWorkbenchModels.Examples.VehicleDrivetrains.BaseConfigurations;
model BaseMiniAFM "Basic architecture of the miniAFM"
  extends VehicleArchitectures.BaseArchitecture(
    redeclare VDCWorkbenchModels.Data.MiniAFMChassis data,
    vehicle(
      N=data.m_vehicle*9.81/4,
      axleFront(steeringRatio=0.9)));
  extends Modelica.Icons.Example;

end BaseMiniAFM;
