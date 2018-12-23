.. _install:

Installing
----------

For production
~~~~~~~~~~~~~~

::

   pip3 install .

or

::

   python3 setup.py install

For development
~~~~~~~~~~~~~~~

::

   pip3 install -e .

or

::

   python3 setup.py develop

The only requirement is
`setuptools <https://pypi.org/project/setuptools/>`__ package, which is
usually a defacto standard in a python3 installation.

Install with docker
~~~~~~~~~~~~~~~~~~~

.. code:: bash

   docker build -t aztarna_docker .