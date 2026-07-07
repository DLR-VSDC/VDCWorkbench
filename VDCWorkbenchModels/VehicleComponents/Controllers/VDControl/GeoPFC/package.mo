within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl;
package GeoPFC "Collection of classes for geometric path following control approach"
  extends Modelica.Icons.Package;

  annotation (
    Documentation(
      info="<html>
<p>
A collection of &quot;geometric&quot; path following control approaches (geoPFC).  
The goal of the geoPFC is to drive two errors to zero:
</p>
<ol>
  <li>
    the orientation error, defined as the difference between the path orientation and
    the vehicle orientation and
  </li>
  <li>
    the lateral error, which is the lateral deviation from the reference path.
  </li>
</ol>
</html>"));
end GeoPFC;
