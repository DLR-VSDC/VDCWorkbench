within VDCWorkbenchModels.UsersGuide;
class ReleaseNotes "Release notes"

  extends Modelica.Icons.ReleaseNotes;

  annotation (Documentation(info="<html>
<p>
This section summarizes the changes that have been performed
on the library.
</p>

<h4>Version 0.3.0 (2026-08-21)</h4>
<p>
This version requires:
</p>
<ul>
  <li>
    Modelica 4.0.0 Library,
  </li>
  <li>
    <a href=\"https://github.com/DLR-SR/Credibility\">Credibility 0.2.0</a> Library,
  </li>
  <li>
    <a href=\"https://github.com/dzimmer/PlanarMechanics\">PlanarMechanics 1.6.0</a> Library,
  </li>
  <li>
    <a href=\"https://github.com/xrg-simulation/SMArtInt\">SMArtInt</a> Library.
    <em>OpenModelica users</em>, please use the branch <a href=\"https://github.com/xrg-simulation/SMArtInt/tree/dev\">dev</a>
    as long as there is no SMArtInt release later then v0.5.2 since there is a&nbsp;bug when using SMArtInt for
    multi-dimensional problems as in <a href=\"modelica://VDCWorkbenchModels.FMUs.DRLAgents.ResidualDRLgeoPFC\">ResidualDRLgeoPFC</a>.
  </li>
  <li>
    <a href=\"https://github.com/modelica/VehicleInterfaces\">VehicleInterfaces 2.0.1</a> Library,
  </li>
</ul>

<p>Improvements:</p>
<ul>
  <li>
    Documentation of several models.
  </li>
</ul>

<p>New components:</p>
<ul>
  <li>
    <a href=\"modelica://VDCWorkbenchModels.FMUs.DRLAgents.ResidualDRLgeoPFC\">Controller model</a> using residual reinforcement learning.
  </li>
  <li>
    <a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.StanleyBased\">Stanley-based controllers</a>.
  </li>
  <li>
    Data for DLR&apos;s <a href=\"modelica://VDCWorkbenchModels.Data.MiniAFMChassis\">miniAFM</a> platform.
  </li>
</ul>


<h4>Version 0.2.0 (2025-08-31)</h4>
<p>
First version of the library to be published.
</p>


<h4>Version 0.1.0 (2025-07-31)</h4>
<p>
First development version of the library.
It requires packages Modelica&nbsp;4.0.0, VehicleInterfaces&nbsp;2.0.1 and PlanarMechanics&nbsp;1.6.0.
</p>
</html>"));
end ReleaseNotes;
