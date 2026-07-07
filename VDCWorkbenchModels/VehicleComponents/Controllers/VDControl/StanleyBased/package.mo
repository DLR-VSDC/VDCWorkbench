within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
package StanleyBased "Collection of Stanley-based control algorithms"
  extends Modelica.Icons.Package;

  annotation (
    Documentation(
      info="<html>
<p>
A collection of Stanley-based path following controllers.
In general, this type of controller minimizes the lateral path deviation and orientation error
at the center of front axle similar to the
<a href=\"modelica://VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.GeoPFC\">geometric path-following controller</a>.
For more information regarding the implemented Stanley controllers, refer to
[<a href=\"modelica://VDCWorkbenchModels.UsersGuide.References\">Brembeck2026</a>].
</p>
</html>"));
end StanleyBased;
