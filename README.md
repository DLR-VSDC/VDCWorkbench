# VDC Workbench: Open-Source Modelica Library for Vehicle Dynamics and Control - VPPC OJVT 2026
## Introduction
VDCWorkbench is an open-source Modelica library for modeling, simulation, and control of electric and software-defined vehicles (SDVs). It provides a unified framework to integrate mechanical, electrical, thermal, and control domains, enabling the development of advanced vehicle architectures from battery-electric and hybrid powertrains to over-actuated by-wire systems. The library supports modular customization of components (tires, actuators, energy storage) and includes state-of-the-art controllers for path following, energy management, and AI-driven control.

The library in this branch was submitted as suplementary for the sumbission to the [2026 Joint Submission of papers to Vehicle Power Propulsion Conference (VPPC)
and IEEE Open Journal of Vehicular Technology (OJVT)](https://events.vtsociety.org/vppc2026/authors/joint-submission-for-ieee-vppc-2026-and-ieee-ojvt/).

## Dependencies
In order to work properly, the library requires the following Modelica packages.
- [Credibility](https://github.com/DLR-SR/Credibility)
- [SMArtIInt](https://github.com/xrg-simulation/SMArtIInt)
- [VehicleInterfaces](https://github.com/modelica/VehicleInterfaces)
- [PlanarMechanics](https://github.com/dzimmer/PlanarMechanics)

Consult the library user's guide for particular versions of the abovementioned packages which are needed.

## Tool compatibility 
The current branch release was developed/tested using following tools.
- [Dymola 2026x Refresh 1](https://www.3ds.com/products-services/catia/products/dymola/): The library has been developed using Dymola.
- In progress: to be tested in [Open Modelica v1.26.x](https://www.openmodelica.org/)

## Bibliography    
- J. Brembeck, R. de Castro, J. Tobol&aacute;&rcaron; and I. Ebrahimi:
IEEE VTS Motor Vehicles Challenge 2023: A Multi-physical Benchmark Problem for Next Generation Energy Management Algorithms, 
*19th IEEE Vehicle Power and Propulsion Conference (VPPC)*, 2022
- J. Brembeck, R. de Castro, J. Ultsch, J. Tobolar, Ch. Winter and K. Ahmic:
VDCWorkbench: A Vehicle Dynamics Control Test &amp; Evaluation Library for Model and AI-based Control Approaches,
accepted for the *16th International Modelica and FMI Conference*, Lucerne, Switzerland, 2025

## License
Copyright &copy; 2022-2026 DLR & UCM. 
The code is released under the [CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode).
Link to [short summary of CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/). For attribution see also [license file](LICENSE.md).
