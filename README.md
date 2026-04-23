# VDC Workbench
The VDCWorkbench Modelica Library is a holistic platform for developing, testing, and validating vehicle dynamics controllers and hybrid vehicle energy management algorithms.

The library was submitted for the "Call for Libraries" at the [Modelica Conference 2025](https://modelica.org/events/modelica2025/).

## Introduction
The ability to systematically compare and evaluate diverse control strategies is essential for the development of effective control algorithms in autonomous driving. To facilitate this, the contribution introduces the VDCWorkbench Modelica Library, a holistic platform for developing, testing, and validating vehicle dynamics controllers and hybrid vehicle energy management algorithms. The presented Library is an extension of the IEEE VTS Motor Vehicle Challenge 2023 models and offers multi-physical component modeling, including a battery with aging model, as well as vehicle dynamics control for autonomous driving research projects. Two path-following approaches are featured: an open-loop lateral controller with a static inversion of a single-track model, and a closed-loop state-dependent geometric path-following controller with static control allocation. The library may also serve as the foundation for development of vehicle control methods, such as two-degree-of-freedom control approaches concepts. One example for this is the combination of a feedforward controller combined with residual reinforcement learning, where a learned agent improves the performance of the open loop controller.
<img width="901" height="982" alt="image" src="https://github.com/user-attachments/assets/1bd548ef-712a-43f8-a27a-67dcfef34c46" />

## Dependencies
In order to work properly, the library requires the following Modelica packages.
- [VehicleInterfaces](https://github.com/modelica/VehicleInterfaces)
- [PlanarMechanics](https://github.com/dzimmer/PlanarMechanics)

Consult the library user's guide for particular versions of the abovementioned packages which are needed.

## Tool compatibility 
The current release was developed/tested using following tools.

- [Dymola 2025x Refresh&nbsp;1](https://www.3ds.com/products-services/catia/products/dymola/): The library has been developed using Dymola.
- [Open Modelica v1.25.0](https://www.openmodelica.org/): The library was tested and is fully compatible to Open Modelica.  
- [Modelon Impact](https://www.modelon.com/modelon-impact/): The library is reported to be fully compatible to Modelon Impact.

## Reference results
The reference results for regression testing can be found in [VDCWorkbench_ReferenceResults](https://github.com/DLR-VSDC/VDCWorkbench_ReferenceResults).

## Bibliography
- J. Brembeck, R. de Castro, J. Tobol&aacute;&rcaron; and I. Ebrahimi:
IEEE VTS Motor Vehicles Challenge 2023: A Multi-physical Benchmark Problem for Next Generation Energy Management Algorithms, 
*19th IEEE Vehicle Power and Propulsion Conference (VPPC)*, 2022
- J. Brembeck, R. de Castro, J. Ultsch, J. Tobolar, Ch. Winter and K. Ahmic:
VDCWorkbench: A Vehicle Dynamics Control Test &amp; Evaluation Library for Model and AI-based Control Approaches,
accepted for the *16th International Modelica and FMI Conference*, Lucerne, Switzerland, 2025

## License
Copyright &copy; 2022-2025 DLR & UCM. 
The code is released under the [CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode).
Link to [short summary of CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/). For attribution see also [license file](LICENSE.md).
