Tools
============

System requirements
============

The tools have is developed for and tested on only **Ubuntu 20.04**

Setup your PC
============
1. Clone bukapilot into your home directory:
``` bash
cd ~
git clone https://github.com/kommuai/bukapilot.git
```

2. Run the setup script:

Ubuntu 20.04 LTS:
``` bash
openpilot/tools/ubuntu_setup.sh
```

3. Ensure you have a working OpenCL runtime:

You can verify your OpenCL installation with the `clinfo` command.

If you do not have any working platforms, you can download drivers from your GPU vendor's site.
On Ubuntu you can just install one of the packages returned by `apt search opencl-icd`.

4. Activate the Python environment:

Execute the following command in root openpilot directory:
```bash
pipenv shell
```

Your shell prompt should change to something similar to `(bukapilot) user@machine:~/bukapilot$ `.

5. Build openpilot by running SCons in the root of the openpilot directory
``` bash
cd bukapilot && scons -j$(nproc)
```

6. Try out some tools!

NOTE: you can always run `update_requirements.sh` to pull in new python dependencies.


[Plot logs](plotjuggler)
-------------

Easily plot openpilot logs with [PlotJuggler](https://github.com/facontidavide/PlotJuggler), an open source tool for visualizing time series data.


[Run openpilot in a simulator](sim)
-------------

Test openpilots performance in a simulated environment. The [CARLA simulator](https://github.com/carla-simulator/carla) allows you to set a variety of features like:
* Weather
* Environment physics
* Cars
* Traffic and pedestrians


[Replay a drive](replay)
-------------

Review video and log data from routes and stream CAN messages to your device.


[Debug car controls](joystick)
-------------

Use a joystick to control your car.


Welcomed contributions
=============

* Documentation: code comments, better tutorials, etc
* Support for platforms other than Ubuntu 20.04
* Performance improvements
* More tools: anything that you think might be helpful to others.

![Imgur](https://i.imgur.com/IdfBgwK.jpg)
