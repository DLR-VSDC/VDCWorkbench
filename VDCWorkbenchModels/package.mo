within ;
package VDCWorkbenchModels
  "Library of models for Modelica Conference Paper VDCWorkbench 2025"
  extends Modelica.Icons.Package;

  annotation (
    preferredView = "info",
    version="0.3.0",
    versionDate = "2025-08-31",
    dateModified = "2025-09-29",
    uses(
      Modelica(version="4.1.0"),
      ModelicaServices(version="4.1.0"),
      PlanarMechanics(version="2.0.0"),
      VehicleInterfaces(version="2.0.2")),
    Documentation(
      revisions="<html>
<table border=\"0\" cellspacing=\"0\" cellpadding=\"2\">
  <tr>
    <td valign=\"middle\">
      <img src=\"modelica://VDCWorkbenchModels/Resources/Images/logo_dlr.png\" width=\"60\">
    </td>
    <td valign=\"middle\">
      &nbsp;&nbsp;&nbsp;
    </td>
    <td valign=\"middle\"><strong>Copyright</strong>
      <br>&copy; <strong>Department of Vehicle System Dynamics and Control, DLR Institute of Vehicle Concepts</strong> and <strong>Dept. of Mechanical Engineering, University of California Merced</strong>
    </td>
  </tr>
</table>
</html>",
      info="<html>
<p>
The models in this library compose vehicle dynamics, powertrain and control modules for
the vehicle benchmark presented at the IEEE VTS Motor Vehicle Challenge 2023. The models
are described in detail in the two papers
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2022</a>].
and
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2025</a>].
</p>
<p>
In order to know how the library works, first have a look at:
</p>
<ul>
  <li>
    <a href=\"modelica://VDCWorkbenchModels.UsersGuide.Tutorial\">Tutorial</a>
    describes the principle ways to use the library.
  </li>
  <li>
    <a href=\"modelica://VDCWorkbenchModels.Examples\">Examples</a>
    contains examples that demonstrate the usage of the library.
  </li>
</ul>

<h4>Licensed by DLR</h4>
<p>
Check the
<a href=\"modelica://VDCWorkbenchModels.UsersGuide.License\">Copyright and License agreement</a>
before using the library.
</p>
</html>"));
end VDCWorkbenchModels;
