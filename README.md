# APOPHISFullSystem
Full-operational system software.

Description: Provides full functionality for all systems. This will be used as the "final code" during full system validation. 
See other repositories such as "gps_test" for specific software for other validation tests.

# Prerequisites
## Software
* Code Composer Studio v7.0 (TI ARM Compiler v16.9.LTS)
* TivaWare for C Series v2.1 (full with driverlib and sensorlib)
## Hardware
* EK-TM4C1294XL LaunchPad (hardware)
* Sensors and etc.

# Installation
1. Add variable SW_ROOT in CCS Workspace. (For example via import vars.ini file with SW_ROOT = C:\Ti\TivaWare_C_Series-2.1.4.178 )
2. Import project into Workspace (Project -> Import CCS Project)
3. Build.

