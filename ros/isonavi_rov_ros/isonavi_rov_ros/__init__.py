"""ROS 2 nodes for isonavi-rov.

The nodes are thin: each wraps a module from the simulation package, so the
code that flies the vehicle is the code the course runs verified. Put the
repository's simulation directory on PYTHONPATH and set ISONAVI_ROOT to the
repository before launching.
"""
