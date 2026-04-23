within VDCWorkbenchModels.VehicleArchitectures;
partial model BaseArchitecture "Basic partial architecture of a planar vehicle"

  parameter Modelica.Units.SI.Velocity v_Start=2 "Initial velocity";
  inner PlanarMechanics.PlanarWorld planarWorld(constantGravity={0,0})
    annotation (Placement(transformation(extent={{80,80},{100,100}})));
  VehicleComponents.Chassis.PlanarVehicle vehicle(
    v_Start=v_Start,
    trackWidth=data.trackWidth,
    R0=data.R0,
    vAdhesion=data.vAdhesion,
    vSlide=data.vSlide,
    mu_A=data.mu_A,
    mu_S=data.mu_S,
    J_wheel=data.J_wheel,
    J_steer=data.J_steer,
    wheelBase=data.wheelBase,
    Jz=data.Jz_vehicle,
    m=data.m_vehicle,
    s=data.s,
    N=data.m_vehicle*9.81/4,
    c_W=data.c_W,
    area=data.area,
    carBody(
      body(
        stateSelect=StateSelect.always,
        r(each fixed=true),
        v(each fixed=true),
        phi(fixed=true),
        w(fixed=true))),
    axleRear(
      wheelLeft(phi_roll(fixed=true, start=0))),
    axleFront(
      wheelLeft(phi_roll(fixed=true, start=0)),
      inertiaSteering(stateSelect=StateSelect.always)))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Utilities.Interfaces.ControlBus controlBus annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={100,0}), iconTransformation(
        extent={{0,0},{0,0}},
        rotation=-90,
        origin={100,80})));
  Data.ROMOParameters data annotation (Placement(transformation(extent={{40,80},{60,100}})));
protected
  VehicleInterfaces.Interfaces.ChassisControlBus chassisControlBus
    annotation (Placement(transformation(extent={{70,50},{90,70}}), iconTransformation(extent={{0,20},{20,40}})));
equation
  connect(vehicle.controlBus, controlBus) annotation (Line(
      points={{10,8},{30,8},{30,0},{100,0}},
      color={255,204,51},
      thickness=0.5));
  connect(chassisControlBus, controlBus.chassisControlBus) annotation (Line(
      points={{80,60},{100,60},{100,-0.1},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  annotation (
    Documentation(info="<html>
<p>
This is a&nbsp;template for a&nbsp;vehicle model. It can be further extended e.g. by
a&nbsp;drive train or a&nbsp;braking assembly.
</p>

<p>
Note: this architecture is <em>partial</em> since there is missing a&nbsp;signal for
the vehicle&apos;s steering wheel. This signal is provided by the <code>steeringWheelAngle</code>
on the <code>chassisControlBus</code>, see also
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Chassis.PlanarVehicle\">PlanarVehicle</a>.
The signal must be connected to the <code>chassisControlBus</code> in the model which
extends this architecture from.
</p>
</html>"));
end BaseArchitecture;
