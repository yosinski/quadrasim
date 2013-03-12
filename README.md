QuadraSim
=========

[QuadraSim](https://github.com/yosinski/quadrasim) is a simulator used in the [Robotics and Intelligent Systems (ROBIN)](http://www.mn.uio.no/ifi/english/research/groups/robin/) group and the [Cornell Creative Machines Laboratory](http://creativemachines.cornell.edu/) to simulate gaits for a research project by Suchan Lee, [Jason Yosinski](http://yosinski.com), and [Jeff Clune](http://jeffclune.com). The simulator was originally written by [Kyrre Glette](http://folk.uio.no/kyrrehg) of the University of Oslo.



Installing the Simulator in Linux
-------------------------------------------

First download the Nvidia PhysX engine for Linux.
This website instructs how to install PhysX in linux (PhysX is primarily supported on Windows): http://about.something.pl/art/howto-setup-physx-sdk-in-linux.html

You must also install and link the necessary boost libraries, Nvidia Cg Toolkit, dynamixel libraries, jpeg-8d libraries, and (maybe) galib247 library.


Then in shell, use the command

    make clean all

to compile the simulator

To use the simulator place the data/ directory which contains the robot information inside the simulator directory, then use the command

    ./name_of_simulator 

to open the simulator.

To input a gait, use the command

    ./name_of_simulator -i gait_file_name

To input a gait and specify the name of the output the results of that gaitfile, use the command

    ./name_of_simulator -i gait_file_name -o output_file_name


To record a video of the gaits (non-headless simulator), you can press TAB, which will generate photos of the gait, which you could put together as a video using ffmpeg or virtualdubmod.



Installing the Simulator in Windows
------------------------------------------

Use the standalone file.
