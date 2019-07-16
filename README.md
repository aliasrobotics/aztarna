# aztarna
<a href="http://www.aliasrobotics.com"><img src="https://aliasrobotics.com/images/logo.png" align="left" hspace="8" vspace="2" width="200"></a>

This repository contains Alias Robotics' aztarna, a footprinting tool for robots.

**Alias Robotics supports original robot manufacturers assessing their security and improving their quality of software. By no means we encourage or promote the unauthorized tampering with running robotic systems. This can cause serious human harm and material damages.**

[![PyPI version](https://badge.fury.io/py/aztarna.svg)](https://badge.fury.io/py/aztarna)   [![Documentation Status](https://readthedocs.org/projects/aztarna/badge/?version=latest)](https://aztarna.readthedocs.io/en/latest/?badge=latest)    [![Article](https://img.shields.io/badge/article-arxiv%3A1812.09490-red.svg)](https://arxiv.org/pdf/1812.09490.pdf)


Aztarna's ROS2 and ROS-Industrial modules are an effort part of the RedROS-I and RedROS2-I FTPs funded by [ROSIN](http://rosin-project.eu/) (European Union’s Horizon 2020 research and innovation programme) with the grant agreement No 732287.

<img src="http://rosin-project.eu/wp-content/uploads/2017/03/Logo_ROSIN_CMYK-Website.png"/>



### For ROS
* A list of the ROS nodes present in the system (Publishers and Subscribers)
* For each node, the published and subscribed topis including the topic type
* For each node, the ROS services each of the nodes offer
* A list of all ROS parameters present in the Parameter Server
* A list of the active communications running in the system. A single communication includes the involved publiser/subscriber nodes and the topics

### For SROS
* Determining if the system is a SROS master.
* Detecting if demo configuration is in use.
* A list of the nodes found in the system. (Extended mode)
* A list of allow/deny policies for each node.
  * Publishable topics.
  * Subscriptable topics.
  * Executable services.
  * Readable parameters.
  
  
### For ROS2 **(Funded under the [ROSIN project](http://rosin-project.eu/))**
* A list of ROS2 nodes present in each communication domains.
* A list of discovered topics on each communication domain.
* A list of discovered services on each communication domain.
* For each node, the relationship of published and subscribed topics.
* For each node, the services provided by that node.

### For Industrial routers
*  Detecting eWON, Moxa, Sierra Wireless and Westermo industrial routers.
*  Default credential checking for found routers.

### For ROS Industrial packages **(Funded under the [ROSIN project](http://rosin-project.eu/))**
* Detection of ROS Industrial Hosts.
* Manufacturers:
    * ABB
    * Fanuc
    * Kuka
        

## Installing
### For production
Direcly from PyPi
```
pip3 install aztarna
```
or from the repository:
```
pip3 install .
```

### For development
```
pip3 install -e .
```
or
```
python3 setup.py develop
```
**Python 3.6** and the [setuptools](https://pypi.org/project/setuptools/) package is required for installation. 
Python 3.7 is recommended.


### ROS2 Module

For usage of the ROS2 footprinting module a ROS2 installation is required. Source the setup.bash script prior to launch.

### Install with docker
```bash
docker build -t aztarna_docker .
```

### Code usage:

```bash
usage: aztarna [-h] -t TYPE [-a ADDRESS] [-p PORTS] [-i INPUT_FILE]
               [-o OUT_FILE] [-e] [-r RATE] [--shodan] [--api-key API_KEY]

Aztarna

optional arguments:
  -h, --help            show this help message and exit
  -t TYPE, --type TYPE  <ROS/ros/SROS/sros/IROUTERS/irouters/ROSIN/rosin> Scan ROS, SROS
                        hosts or Industrial routers
  -a ADDRESS, --address ADDRESS
                        Single address or network range to scan.
  -p PORTS, --ports PORTS   
                        Ports to scan (format: 13311 or 11111-11155 or
                        1,2,3,4)
  -i INPUT_FILE, --input_file INPUT_FILE
                        Input file of addresses to use for scanning
  -o OUT_FILE, --out_file OUT_FILE
                        Output file for the results
  -e, --extended        Extended scan of the hosts
  -r RATE, --rate RATE  Maximum simultaneous network connections
  --shodan              Use shodan for the scan types that support it.
  --api-key API_KEY     Shodan API Key

```

### Run the code (example input file):

```bash
aztarna -t ROS -p 11311 -i ros_scan_s20.csv
```

### Run the code with Docker (example input file):
```bash
docker run -v <host_path>:/root -it aztarna_docker -t ROS -p 11311 -i <input_file>
```

### Run the code (example single ip address):

```bash
aztarna -t ROS -p 11311 -a 115.129.241.241
```

### Run the code (example subnet):

```bash
aztarna -t ROS -p 11311 -a 115.129.241.0/24
```

### Run the code (example single ip address, port range):

```bash
aztarna -t ROS -p 11311-11500 -a 115.129.241.241
```

### Run the code (example single ip address, port list):

```bash
aztarna -t ROS -p 11311,11312,11313 -a 115.129.241.241
```

### Run the code (example piping directly from zmap):

```bash
zmap -p 11311 0.0.0.0/0 -q | aztarna -t SROS -p 11311
```

### Run the code (example search for industrial routers in shodan)
```bash
aztarna -t IROUTERS --shodan --api-key <yourshodanapikey>
```

### Run the code (example search for industrial routers in shodan, piping to file)
```bash
aztarna -t IROUTERS --shodan --api-key <yourshodanapikey> -o routers.csv
```
## Cite our work
If you're using our work for your research, please cite us as:
```
@ARTICLE{2018arXiv181209490V,
  author = {{Vilches}, V{\'\i}ctor Mayoral and {Mendia}, Gorka Olalde and
  {Baskaran}, Xabier Perez and {Cordero}, Alejandro Hern{\'a}ndez
  and {Juan}, Lander Usategui San and {Gil-Uriarte}, Endika and
  {de Urabain}, Odei Olalde Saez and {Kirschgens}, Laura Alzola},
  title = "{Aztarna, a footprinting tool for robots}",
  journal = {arXiv e-prints},
  keywords = {Computer Science - Cryptography and Security, Computer Science - Robotics},
  year = 2018,
  month = Dec,
  eid = {arXiv:1812.09490},
  pages = {arXiv:1812.09490},
  archivePrefix = {arXiv},
  eprint = {1812.09490},
  primaryClass = {cs.CR},
  adsurl = {https://ui.adsabs.harvard.edu/\#abs/2018arXiv181209490V},
  adsnote = {Provided by the SAO/NASA Astrophysics Data System}
}
```
