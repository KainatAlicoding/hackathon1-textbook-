# Week 5: ROS 2 Packages and Build System

This week, we will understand how to organize and build ROS 2 projects using packages and the Colcon build system. We will cover:

- Structure of a ROS 2 package.
- Creating custom packages.
- Using `package.xml` and `setup.py`/`CMakeLists.txt`.
- Building and installing packages with Colcon.

- Structure of a ROS 2 package.

	A ROS 2 package is the fundamental unit of code organization and distribution. It contains source code, a `package.xml` manifest (metadata and dependencies), and build system files (`CMakeLists.txt` for `ament_cmake` packages or `setup.py`/`pyproject.toml` for `ament_python` packages). Typical layout:

	```
	my_package/
	  package.xml
	  CMakeLists.txt        # for C++/ament_cmake packages
	  setup.py              # for Python/ament_python packages
	  resource/             # package resource marker for Python packages
	  src/                  # source code (C++ or Python modules)
	  launch/               # ROS 2 launch files
	  cfg/                  # configuration
	  test/                 # tests
	```

	Naming and dependency declarations in `package.xml` make packages discoverable by the build system and tooling. Keep one clear responsibility per package when possible (nodes, drivers, stacks, libraries).

- Creating custom packages.

	Create new packages using the `ros2 pkg create` helper. Choose a build type that matches your language and needs: `ament_cmake` for C++ and `ament_python` for Python. Example commands:

	```
	# Python package
	ros2 pkg create --build-type ament_python my_python_pkg

	# C++ package
	ros2 pkg create --build-type ament_cmake my_cpp_pkg
	```

	The generator will scaffold `package.xml` and basic build files. Add your node code under `src/` (for Python: `src/my_python_pkg/`), and update `CMakeLists.txt` or `setup.py` to install entry points for `ros2 run`.

- Using `package.xml` and `setup.py`/`CMakeLists.txt`.

	- `package.xml` declares package metadata (name, version, maintainers) and dependencies (`build_depend`, `exec_depend`, `test_depend`). Be specific with dependency types so colcon and ament can order builds correctly.

	- For Python packages, `setup.py` (or `pyproject.toml`) configures installation, package data, and entry points. Use `console_scripts` entry points to create runnable nodes:

	```python
	# setup.py snippet
	entry_points={
	    'console_scripts': [
	        'talker = my_python_pkg.talker:main',
	    ],
	}
	```

	- For C++ packages, `CMakeLists.txt` defines targets, links libraries, and exports install rules so `ros2 run` can find executables. Use `ament_package()` at the end of the CMake file to register the package with ament.

- Building and installing packages with Colcon.

	ROS 2 uses `colcon` as the recommended build tool. The typical workflow:

	1. Source your ROS 2 installation (example for Windows/PowerShell may differ):

	```powershell
	# Linux example (bash)
	source /opt/ros/<distro>/setup.bash

	# On Windows with ROS 2 installed, use the appropriate setup script in PowerShell
	```

	2. Build the workspace from the workspace root (where `src/` lives):

	```bash
	colcon build --symlink-install
	```

	3. Source the overlay to use newly built packages:

	```bash
	source install/setup.bash
	```

	4. Run nodes from built packages:

	```bash
	ros2 run my_python_pkg talker
	ros2 run my_cpp_pkg my_cpp_node
	```

	Notes: use `--symlink-install` during development for faster iterative edits with Python packages. For Windows PowerShell users, replace `source` commands with the appropriate PowerShell `Dot-Sourcing` commands provided by your ROS 2 distro documentation.

	Useful colcon/ROS commands:

	- `colcon build --packages-select <pkg>` — build a single package.
	- `ros2 pkg list` — list installed/discovered packages.
	- `ros2 pkg prefix <pkg>` — show package install prefix.
	- `ros2 run <pkg> <executable>` — run an installed node.

	Testing and packaging: add unit tests under `test/` and run `colcon test` followed by `colcon test-result --verbose` to inspect failures. To distribute, publish source or binaries following your project's release process and update `package.xml` with license and export information.