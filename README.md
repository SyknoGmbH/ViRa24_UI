# ViRa24 Evaluation GUI
This is the official user interface for Sykno's ViRa24 vital sign sensing evaluation radar system.

# Features
The software is designed to work witht the ViRa24 vital sign sensing radar and covers the following features:
 - Live display of
	 - raw inphase and quadrature radar data
	 - visualization of the IQ diagram
	 - target movement
	 - respiratory motion
	 - pulse movement
	 - heart sounds
	 - external ECG sensor (if connected)
	 - frequency spectrum of raw data
 - Setting of all filter parameters
 - Setting of all system parameters
 - Saving raw data of all channels
 
# Windows executable
For an easy start without an installed python environment, a windows executable is available in the folder *windows_project*.
 
# Python project
The full source code is available in the folder *python_project*. The required packages can be installed with the following command: 
```console 
pip install pip install -r requirements.txt
```

The ViRa24 GUI can be started with the following command:
```console 
python3 main.py
```

Under Linux, *sudo* privileges are required to allow accessing the data port.


# Maintainer
This project is maintained by Sykno GmbH. For more information and inquiries, please visit [www.sykno.de.](www.sykno.de) 

# Disclaimer
This software comes with no warranty. 

# License
The project is licensed under the _GNU General Public License (GPLv3)_. 
