# DRC_Fortran
Delft Research Controller (DRC), Bladed-style DISCON baseline wind turbine controller

The Delft Research Controller (DRC) provides an open, modular and fully adaptable baseline wind turbine controller to the scientiﬁc community. New control implementations can be added to the existing baseline controller, and in this way, convenient assessments of the proposed algorithms is possible. Because of the open character and modular set-up, scientists are able to collaborate and contribute in making continuous improvements to the code. The DRC is being developed in Fortran and uses the Bladed-style DISCON controller interface. The compiled controller is conﬁgured by a single control settings parameter ﬁle, and can work with any wind turbine model and simulation software using the DISCON interface. Baseline parameter ﬁles are supplied for the NREL 5-MW and DTU 10-MW reference wind turbines.

If you want to use the controller with DNV GL Bladed v4.5 or earlier (which still has support for the DISCON external controller interface), do the following:
1. Be sure to use and place the 32-bit DLL in the same folder as where you put your project .$PJ-file
2. Copy in that same folder the DISCON.IN controller configuration file
3. Set-up the 32-bit DLL as an external controller (Control -> Discrete External Controller -> Define...)
3. Open the DISCON.IN file with a text editor and copy its entire contents in the "External controller data:" section (Control -> Discrete External Controller -> Define...)
4. Run a "Power Production" simulation