.. Aztarna documentation master file, created by
   sphinx-quickstart on Wed Sep  5 17:23:49 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

**Alias Robotics supports original robot manufacturers assessing their
security and improving their quality of software. By no means we
encourage or promote the unauthorized tampering with running robotic
systems. This can cause serious human harm and material damages.**

For ROS
-------

-  A list of the ROS nodes present in the system (Publishers and
   Subscribers)
-  For each node, the published and subscribed topis including the topic
   type
-  For each node, the ROS services each of the nodes offer
-  A list of all ROS parameters present in the Parameter Server
-  A list of the active communications running in the system. A single
   communication includes the involved publiser/subscriber nodes and the
   topics

For SROS
--------

-  Determining if the system is a SROS master.
-  Detecting if demo configuration is in use.
-  A list of the nodes found in the system. (Extended mode)
-  A list of allow/deny policies for each node.

   -  Publishable topics.
   -  Subscriptable topics.
   -  Executable services.
   -  Readable parameters.

For ROS2
--------
- Detection of ROS2 nodes in all possible ROS2 domain IDs. Local network.
- Listing of all available topics and their relationship to nodes.
- Listing of all available services and their relationship to nodes.


For Industrial routers
----------------------

-  Detecting eWON, Moxa, Sierra Wireless and Westermo industrial routers.
-  Default credential checking for found routers.







..  toctree::
    :maxdepth: 4
    :caption: Contents:

    install
    usage
    modules

..  automodule:: aztarna
    :members:
    :undoc-members:
    :show-inheritance:



Indices and tables
==================
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

