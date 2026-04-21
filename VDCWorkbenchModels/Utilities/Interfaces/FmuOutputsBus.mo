within VDCWorkbenchModels.Utilities.Interfaces;
expandable connector FmuOutputsBus "FMU outputs signals for DRL training"
  extends Modelica.Icons.SignalSubBus;
  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2},
        initialScale=0.1),
      graphics={
        Rectangle(
          extent={{-8,6},{10,-2}},
          lineColor={255,204,51},
          lineThickness=0.5)}),
    Diagram(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}},
        grid={2,2},
        initialScale=0.1)),
    Documentation(info="<html>
<p>
Contains all signals which are used in for rsidual DRL training.
</p>
</html>"));
end FmuOutputsBus;
