within VDCWorkbenchModels.Utilities.Interfaces;
expandable connector FmuOutputsBus "FMU outputs signals for DRL training"

  annotation (
    Icon(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}, initialScale=0.2),
      graphics={
          Rectangle(
            lineColor={215,136,255},
            lineThickness=0.5,
            extent={{-20,-2},{20,2}}),
          Polygon(
            fillColor={225,165,255},
            fillPattern=FillPattern.Solid,
            points={{-78,50},{82,50},{102,30},{82,-40},{62,-50},{-58,-50},{-78,-40},{-98,30}},
            smooth=Smooth.Bezier),
          Ellipse(
            fillPattern=FillPattern.Solid,
            extent={{-65,15},{-55,25}}),
          Ellipse(
            fillPattern=FillPattern.Solid,
            extent={{-5,15},{5,25}}),
          Ellipse(
            fillPattern=FillPattern.Solid,
            extent={{55,15},{65,25}}),
          Ellipse(
            fillPattern=FillPattern.Solid,
            extent={{-35,-25},{-25,-15}}),
          Ellipse(
            fillPattern=FillPattern.Solid,
            extent={{25,-25},{35,-15}})}),
    Diagram(
      coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}, initialScale=0.2),
      graphics={
        Polygon(
          points={{-40,25},{40,25},{50,15},{40,-20},{30,-25},{-30,-25},{-40,-20},{-50,15}},
          fillColor={215,136,255},
          fillPattern=FillPattern.Solid,
          smooth=Smooth.Bezier),
        Ellipse(
          extent={{-32.5,7.5},{-27.5,12.5}},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-2.5,12.5},{2.5,7.5}},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{27.5,12.5},{32.5,7.5}},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-17.5,-7.5},{-12.5,-12.5}},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{12.5,-7.5},{17.5,-12.5}},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-150,70},{150,40}},
          textString="%name")}),
    Documentation(info="<html>
<p>
Contains all signals which are used in for residual DRL training.
</p>
</html>"));
end FmuOutputsBus;
