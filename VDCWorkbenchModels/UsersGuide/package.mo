within VDCWorkbenchModels;
package UsersGuide "User's Guide"
  extends Modelica.Icons.Information;

  annotation (
    DocumentationClass=true,
    Documentation(
      info="<html>
<p>
Library <strong>VDCWorkbenchModels</strong> is a&nbsp;<strong>free</strong> (check the
<a href=\"modelica://VDCWorkbenchModels.UsersGuide.License\">Copyright and License agreement</a>)
Modelica package providing models of vehicle dynamics, powertrain and control modules
for the vehicle benchmark presented at the IEEE VTS Motor Vehicle Challenge 2023.
The models are described in detail in the two papers
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2022</a>].
and
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2025</a>].
</p>

<p>
The library requires the following packages.
</p>
<ul>
  <li><a href=\"https://github.com/modelica/ModelicaStandardLibrary\">Modelica</a>,</li>
  <li><a href=\"https://github.com/DLR-SR/Credibility\">Credibility</a>,</li>
  <li><a href=\"https://github.com/dzimmer/PlanarMechanics\">PlanarMechanics</a>,</li>
  <li>
    <a href=\"https://github.com/xrg-simulation/SMArtInt\">SMArtInt</a> &ndash;
    <strong>OpenModelica users</strong>: please use the branch
    <a href=\"https://github.com/xrg-simulation/SMArtInt/tree/dev\">dev</a>
    as long as there is no SMArtInt release later then v0.5.2 since there is a&nbsp;bug
    when using SMArtInt for multi-dimensional problems,
  </li>
  <li><a href=\"https://github.com/modelica/VehicleInterfaces\">VehicleInterfaces</a>.</li>
</ul>
<p>
For particular versions of the libraries, please check the
<a href=\"modelica://VDCWorkbenchModels.UsersGuide.ReleaseNotes\">release notes</a>.
</p>

<p>
In this User's Guide the most important aspects of the library are sketched.
</p>
</html>"));
end UsersGuide;
