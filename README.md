# aztarna
A footprinting tool for ROS and SROS systems. The collected information includes the following:

### For ROS
* A list of the ROS nodes present in the system (Publishers and Subscribers)
* For each node, the published and subscribed topis including the topic type
* For each node, the ROS services each of the nodes offer
* A list of all ROS parameters present in the Parameter Server
* A list of the active communications running in the system. A single communication includes the involved publiser/subscriber nodes and the topics

### For SROS
* TODO

## Installing
```
sudo python3 setup.py install
```
The only requirement is [setuptools](https://pypi.org/project/setuptools/) package, which is usually a defacto standard in a python3 installation.
