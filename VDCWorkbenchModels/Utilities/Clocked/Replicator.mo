within VDCWorkbenchModels.Utilities.Clocked;
block Replicator "Replicate a periodic clock signal"
  extends Modelica.Clocked.ClockSignals.Interfaces.ClockedBlockIcon;

  parameter Integer nout=1 "Number of outputs";

  Modelica.Clocked.ClockSignals.Interfaces.ClockInput u "Connector of clocked input signal" annotation (Placement(
        transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Clocked.ClockSignals.Interfaces.ClockOutput y[nout] "Connector of clocked output signals" annotation (Placement(
        transformation(extent={{100,-10},{120,10}})));

equation
  y = fill(u, nout);

  annotation (
    Icon(coordinateSystem(
        preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}),
      graphics={
        Line(
          points={{-100,0},{100,0}},
          color={95,95,95}),
        Line(
          points={{100,-10},{0,0},{100,10}},
          color={95,95,95}),
        Ellipse(
          extent={{-15,15},{15,-15}},
          color={95,95,95},
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid)}),
    Documentation(info="<html>
<p>
This block replicates the input periodic clock signal to an array of <code>nout</code> identical output signals.
The block is similar to the block in <a href=\"modelica://Modelica.Blocks.Routing.Replicator\">Modelica.Blocks.Routing.Replicator</a>,
but adapted to work in clocked partitions.
</p>
</html>"));
end Replicator;
