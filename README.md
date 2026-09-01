# VDCWorkbench: A Source-Available Modelica Toolbox for Research and Education on Vehicle Dynamics and Control
## Introduction
VDCWorkbench is a source-available Modelica library for modeling, simulation, and control of electric and software-defined vehicles (SDVs). It provides a unified framework to integrate mechanical, electrical, thermal, and control domains, enabling the development of advanced vehicle architectures from battery-electric and hybrid powertrains to over-actuated by-wire systems. The library supports modular customization of components (tires, actuators, energy storage) and includes state-of-the-art controllers for path following, energy management, and AI-driven control.
<img width="2830" height="4014" alt="Library_Overview_OJVT2026_v2" src="https://github.com/user-attachments/assets/2a0fcb52-f89f-4456-9c08-32b41a7c35a6" />
*Map based on [BayernAtlas](https://atlas.bayern.de) (&copy; Bayerische Vermessungsverwaltung 2026)*

The library was submitted as suplementary for the sumbission to the [2026 Joint Submission of papers to Vehicle Power Propulsion Conference (VPPC)
and IEEE Open Journal of Vehicular Technology (OJVT)](https://events.vtsociety.org/vppc2026/authors/joint-submission-for-ieee-vppc-2026-and-ieee-ojvt/).

The full article is available here as open access: [VDCWorkbench: A Source-Available Modelica Toolbox for Research and Education on Vehicle Dynamics and Control](https://ieeexplore.ieee.org/document/11573019).


## Dependencies
In order to work properly, the library requires the following Modelica packages.
- [Credibility](https://github.com/DLR-SR/Credibility)
- [PlanarMechanics](https://github.com/dzimmer/PlanarMechanics)
- [SMArtInt](https://github.com/xrg-simulation/SMArtInt)
- [VehicleInterfaces](https://github.com/modelica/VehicleInterfaces)

Consult the VDCWorkbenchModels Library user's guide for particular versions of the abovementioned packages which are needed.

## Tool compatibility
The current branch release was developed/tested using following tools.
- [Dymola 2026x Refresh 1](https://www.3ds.com/products-services/catia/products/dymola/): The library has been developed using Dymola.
- [OpenModelica v1.27.0 (64-bit)](https://www.openmodelica.org/): The library was tested and is fully compatible to Open Modelica.

## Bibliography
- J. Brembeck, R. de Castro, J. Tobol&aacute;&rcaron; and I. Ebrahimi: IEEE VTS Motor Vehicles Challenge 2023: A Multi-physical Benchmark Problem for Next Generation Energy Management Algorithms, *19th IEEE Vehicle Power and Propulsion Conference (VPPC)*, 2022
- J. Brembeck, R. de Castro, J. Ultsch, J. Tobolar, Ch. Winter and K. Ahmic: VDCWorkbench: A Vehicle Dynamics Control Test &amp; Evaluation Library for Model and AI-based Control Approaches, *16th International Modelica and FMI Conference*, Lucerne, Switzerland, doi: [10.3384/ecp218585](https://doi.org/10.3384/ecp218585), 2025
- J. Brembeck et al.: VDCWorkbench: A Source-Available Modelica Toolbox for Research and Education on Vehicle Dynamics and Control, in *IEEE Open Journal of Vehicular Technology*, doi: [10.1109/OJVT.2026.3705616](https://doi.org/10.1109/OJVT.2026.3705616), 2026

## License
Copyright &copy; 2022-2026 DLR & UCM.
The code is released under the [CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode).
Link to [short summary of CC BY-NC-ND 4.0 license](https://creativecommons.org/licenses/by-nc-nd/4.0/). For attribution see also [license file](LICENSE.md).

## Appendix: Nomenclature
Download as pdf: [ovjt4-Nomenclature.pdf](https://github.com/user-attachments/files/28633609/ovjt4-Nomeclature.pdf)

<img width="4961" height="6111" alt="ovjt4-Nomeclature" src="https://github.com/user-attachments/assets/c6dc5d56-9c16-457a-a735-146531e405b1" />

