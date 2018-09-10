.. _usage:

Usage:
------

.. code:: bash

   usage: aztarna [-h] -t TYPE [-a ADDRESS] -p PORTS [-i INPUT_FILE]
                  [-o OUT_FILE] [-e EXTENDED]

   Aztarna

   optional arguments:
     -h, --help            show this help message and exit
     -t TYPE, --type TYPE  <ROS/SROS> Scan ROS or SROS hosts
     -a ADDRESS, --address ADDRESS
                           Single address or network range to scan.
     -p PORTS, --ports PORTS
                           Ports to scan (format: 13311 or 11111-11155 or
                           1,2,3,4)
     -i INPUT_FILE, --input_file INPUT_FILE
                           Input file of addresses to use for scanning
     -o OUT_FILE, --out_file OUT_FILE
                           Output file for the results
     -e EXTENDED, --extended EXTENDED
                           Extended scan of the hosts
     -r RATE, --rate RATE
                           Maximum simultaneous network connections

Run the code (example input file):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   aztarna -t ROS -p 11311 -i ros_scan_s20.csv

Run the code with Docker (example input file):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   docker run -v <host_path>:/root -it aztarna_docker -t ROS -p 11311 -i <input_file>

Run the code (example single ip address):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   aztarna -t ROS -p 11311 -a 115.129.241.241

Run the code (example subnet):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   aztarna -t ROS -p 11311 -a 115.129.241.0/24

Run the code (example single ip address, port range):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   aztarna -t ROS -p 11311-11500 -a 115.129.241.241

Run the code (example single ip address, port list):
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: bash

   aztarna -t ROS -p 11311,11312,11313 -a 115.129.241.241
