HuMoD Database
===
The HuMoD Database is a versatile and open database for the investigation, modeling and simulation of human motion dynamics with a focus on lower limbs. The database contains raw and processed biomechanical measurement data from a three-dimensional motion capture system, an instrumented treadmill and an electromyographical measurement system for eight different motion tasks performed by a representative female and male subject as well as anthropometric information for both subjects. This repository provides the source-code of the applied computational scripts licensed under BSD 3-Clause License. The database can be found on the [HuMoD Database website](http://www.sim.informatik.tu-darmstadt.de/res/ds/humod/ "HuMoD Database").

Please run [DirectoryStructureGeneration.m](DirectoryStructureGeneration.m) first and modify the global path in [Scripts/getPath.m](Scripts/getPath.m) and the local paths in [Scripts/getFile.m](Scripts/getFile.m) if required.

Main features:
---
* Synchronization and filtering of raw kinematic motion, ground reaction force and muscle activity data
* Estinmation of anthropometric parameters
* Estimation of joint center positions
* Estimation of joint angle trajectories
* Decomposition of ground reaction forces during double support phase
* Detection of step events
* Estimation of center of pressure trajectories
* Visualization of processed kinematic motion, ground reaction force and muscle activity data
