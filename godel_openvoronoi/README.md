# Godel project-specific OpenVoronoi fork

This repository contains a ROS-Industrial Godel project-specific fork of the OpenVoronoi library written by Anders Wallin.

It has been converted to a ROS catkin package by adding a package manifest (`package.xml`) and a replacement build script (`CMakeLists.txt`). It has additionally been patched to provide a getter for the `MachiningGraph` member variable `g` in the `OffsetSorter` class.

Apart from these changes (and this new `README.md` file), all sources and other files have been left unchanged and are identical to those in the [source repository][].


## Updates

Upstream is tracked in the `upstream` branch. In order to update the ROS wrapper package, checkout the branch corresponding to the targeted ROS distribution (ie: `hydro-devel` for ROS Hydro), then rebase it onto `upstream`. Resolve any conflicts, then (force) push to the distribution branch.

Alternatively, `upstream` could be merged into the target distribution branch. Again, push to the distribution branch.


## Releases

It should be possible to treat the resulting `godel_openvoronoi` package as just another ROS package. It should not require any special consideration or steps while releasing.



[source repository]: https://github.com/aewallin/openvoronoi
