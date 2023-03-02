## Overview

This source code provides a complete example application for viewing amc/asf
motion capture data.

## Controls

The application reads data from a directory tree, expecting one asf file and
possibly several amc files per directory. Once the motion has been loaded,
a
* PAGEUP/PAGEDOWN selects motion for playback. On the Mac, hold down the fn key and type the up/down arrow keys.

* SPACE to toggle 0x (pause), 0.1x (slow), 1x (normal) speed

* TAB to toggle camera tracking, 

* ESC to quit

* mouse buttons for camera control. One the Mac, the three mouse
buttons are obtained using the normal button, the normal button while
holding down the option key, and the normal button while holding down
the "Apple" key.

* 'd' will dump all the currently loaded motions to associated .global files. In such a file, each line contains a reference position of the character on the floor (position.x, position.z) and a reference yaw (position.yaw),
followed by various joint positions in this reference coordinate frame.
(There is a line for each frame of the motion.)

Using the function defined in Vector/Misc.hpp, that means that, for example,
the root position is given:

```
rotate_by_yaw(root_from_file, position.yaw) + make_vector(position.x, 0.0, position.z)
```

In other words, to go from the root.xyz in the file (root_from_file) to the
global root position, you'd use the following formulas:

```
root.x = root_from_file.x * cos(position.yaw) + root_from_file.z * sin(position.yaw) + position.x
root.y = root_from_file.y
root.z = root_from_file.z * cos(position.yaw) + root_from_file.x * -sin(position.yaw) + position.z
```

## Compiling

### 1. Windows

The environment has been setup for windows under VS2017. Click ide/VS2017/amc_viewer.sln and run.

### 2. Ubuntu 18.04

Step 1. Install dependencies (sdl1.2.15-dev, libxml2-dev) and compiling tools (jam)
```
sudo apt-get install alien
mkdir tmp
cd tmp
wget https://www.libsdl.org/release/SDL-devel-1.2.15-1.x86_64.rpm
sudo alien -i SDL-devel-1.2.15-1.x86_64.rpm
cd ..
rm -r tmp
sudo apt-get install libxml2-dev
sudo apt-get install jam
```

Step 2. Compile with jam
```
jam
```

Step 3. Test
```
cd dist
./browser
```

## Data
- CMU Mocap http://mocap.cs.cmu.edu/.
- Mixamo https://www.mixamo.com/#/

## Copyright
All source is copyright Jim McCann unless otherwise noted. Feel free to use
in your own projects (please retain some pointer or reference to the original
source in a readme or credits section, however).

Contributing back bugfixes and improvements is polite and encouraged but not
required.

## Contact
Written by:
Jim McCann (jmccann@cs.cmu.edu) [http://tchow.com] <br/> Additional text by Roger Dannenberg (rbd@cs.cmu.edu) <br/>Feel free to email.
