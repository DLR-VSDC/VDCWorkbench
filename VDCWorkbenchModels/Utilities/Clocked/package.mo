within VDCWorkbenchModels.Utilities;
package Clocked
  "Clock triggered blocks for describing synchronous behavior suited for implementation of control systems"
  extends Modelica.Icons.Package;

  annotation (
    Documentation(
      info="<html>
<p>
Collection of models to synchronize sampled data systems with different sampling rates which are yet
not contained in <a href=\"modelica://Modelica.Clocked\">Modelica.Clocked</a>.
</p>
</html>"),
    Icon(
      graphics={
        Ellipse(extent={{-80,-80},{80,80}}),
        Line(points={{80,0},{60,0}}),
        Line(points={{69.282,40},{51.962,30}}),
        Line(points={{40,69.282},{30,51.962}}),
        Line(points={{0,80},{0,60}}),
        Line(points={{-40,69.282},{-30,51.962}}),
        Line(points={{-69.282,40},{-51.962,30}}),
        Line(points={{-80,0},{-60,0}}),
        Line(points={{-69.282,-40},{-51.962,-30}}),
        Line(points={{-40,-69.282},{-30,-51.962}}),
        Line(points={{0,-80},{0,-60}}),
        Line(points={{40,-69.282},{30,-51.962}}),
        Line(points={{69.282,-40},{51.962,-30}}),
        Line(points={{80,0},{60,0}}),
        Line(points={{0,0},{-50,50}}),
        Line(points={{0,0},{40,0}})}));
end Clocked;
