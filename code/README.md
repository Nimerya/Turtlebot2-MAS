System structure:
- the `Controller.py` script must be used as a main for the application.
- the `Terminal` package contains the scripts that are responsible for the instantiation of the terminals (for logging purpouses).
- the `pipes` folder is used to store the pipes that are used to communicate with the terminals.
- the `LindaProxy` package contains the implementation of a proxy that translates the messages that are incoming from python in a way that DALI can understand.
- `DALI` contains the DALI subsystem.