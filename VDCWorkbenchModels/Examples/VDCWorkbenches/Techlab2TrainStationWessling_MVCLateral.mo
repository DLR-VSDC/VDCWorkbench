within VDCWorkbenchModels.Examples.VDCWorkbenches;
model Techlab2TrainStationWessling_MVCLateral "Vehicle architecture for Motor Vehicles Challenge with vehicle dynamics control based on model inversion"
  extends VehicleArchitectures.VDCWorkbench2025(
    redeclare VehicleComponents.Controllers.VDControl.MVCLateralControl controller(
      filePath=ModelicaServices.ExternalReferences.loadResource(
        "modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt.mat"),
      pathName="path"));
  annotation (
    experiment(
      StopTime=136,
      Interval=0.01,
      __Dymola_Algorithm="Dassl"),
    Documentation(
      info="<html>
<p>
Vehicle&apos;s architecture used for the IEEE VTS Motor Vehicles Challenge 2023 using
a&nbsp;longitudinal and lateral control algorithm to control planar behavior of the vehicle.
The controller follows the route from the DLR&apos;s site in Oberpfaffenhofen, Germany, to
the next railroad station in Wessling.
</p>
</html>",
      figures = {
        Figure(
          identifier = "Techlab2TrainStationWessling_MVCLateral_results",
          preferred = true,
          plots = {
            Plot(
              title = "Vehicle velocity",
              identifier = "v_vehicle",
              curves = {
                Curve(y = controlBus.chassisBus.longitudinalVelocity, legend = "chassisBus.longitudinalVelocity")}),
            Plot(
              title = "Demanded driving (+) / braking (-) torque",
              identifier = "torque",
              curves = {
                Curve(y = controlBus.electricMotorControlBusFM.torque, legend = "electric motor front central"),
                Curve(y = controlBus.electricMotorControlBusRL.torque, legend = "electric motor rear left"),
                Curve(y = controlBus.electricMotorControlBusRR.torque, legend = "electric motor rear right")}),
            Plot(
              title = "Power loss",
              identifier = "P_loss",
              curves = {
                Curve(y = Ploss)})},
          caption = "%(plot:v_vehicle) Vehicle speed (%(variable:controlBus.chassisBus.longitudinalVelocity)) due to driving / braking torques, shown in %(plot:torque).")}));
end Techlab2TrainStationWessling_MVCLateral;
