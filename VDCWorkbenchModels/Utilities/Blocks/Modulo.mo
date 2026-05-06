within VDCWorkbenchModels.Utilities.Blocks;
block Modulo "Modulo"
  parameter Real k(start=1, unit="1") "Modulo value";

  Modelica.Blocks.Interfaces.RealInput u "Input signal connector" annotation (
      Placement(transformation(extent={{-140,-20},{-100,20}}, rotation=0)));
  Modelica.Blocks.Interfaces.RealOutput y "Output signal connector" annotation (
     Placement(transformation(extent={{100,-10},{120,10}}, rotation=0)));
equation
  y = mod(u,k);
  //y = noEvent(mod(u,k));
  //y = smooth(1, if u<k then u else u-k);

  annotation (
    Documentation(info="<html>
<p>
This block computes output <em>y</em> as
<em>remainder of a division</em> of <var>u</var> (dividend) by <em>k</em> (divisor) as
</p>
<pre>
    y = u mod k;
</pre>

</html>"),
    Icon(
      coordinateSystem(preserveAspectRatio=true,extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(
          points={{-100,-100},{-100,100},{100,0},{-100,-100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-150,-140},{150,-100}},
          textColor={0,0,0},
          textString="k=%k"),
        Text(
          extent={{-80,20},{40,-20}},
          textColor={192,192,192},
          textString="mod"),
        Text(
          extent={{-150,140},{150,100}},
          textString="%name",
          textColor={0,0,255})}),
    Diagram(
      coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,100}})));
end Modulo;
