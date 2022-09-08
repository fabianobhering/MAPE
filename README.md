# MAPE: Multimedia-Aware Performance Estimator


MAPE is a deterministic simulation-based estimator that provides real-time per-flow throughput, packet loss and delay estimates while considering inter-flow interference and multi-rate flows, typical of multimedia traffic.

# Instructions
The root directory contains the source code and the directory data. The directory data must contain all the data of the project, like scenarios, instances, and other input and output of the project. 

* `estimates`: Contains the output files, with the estimates returned in each simulation.
* `etx`: Contains the input files of the network topology, with the and quality of the links (etx).
* `instances`: Contains the instances generated with the different sources and destinations.
* `routes`: Contains output files with routes used as input to ns-3 to validate performance.
* `scenarios`: Contains the scenarios generated with the different number and positioning of the network nodes.

# Building

Clone this repository on your local machine and run:

    make all

# Running

Once MAPE binaries has been linked, simply run the `./mape` `<topology>` `<inst_dir>` found in the root of the project directory.

Examples:

    $ ./mape ext/etxGrid56 instance/instGrid3
 
# Contacts
For further information contact Fabiano Bhering at fabianobhering@cefetmg.br.
