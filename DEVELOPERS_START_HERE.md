# Developer Configuration

Sharkbots uses Python for programming. Because Python has a very large community and because it's very easy to add new libraries to a python environment, it's very easy for developers to end up with a lot of complex dependencies that end up causing trouble on the actual RoboRio or outboard processor. To (help) fight that, please use the **conda** package manager to create a "virtual environment." Everyone will share the same libraries and be able to to recreate the virtual environment quickly.

**NOTE**: At the moment, Larry has had difficulty installing the Python wpilib into Window's implementation of Linux (Windows Subsystem for Linux). Until that's figured out, if you're using a Windows machine, use the **Command prompt** or **Windows Powershell** terminals to work. 

## Clone this repo 

Using your git client, create a local copy of this Github repository on your development machine. If you have a graphical user interface, use it. If you have a command-line version of **git** installed, go to the directory in which you want to do your programming, and run:

    git clone https://github.com/WHEARobotics/FRC2023

This will create a subdirectory called **FRC2023** containing the latest contents of this repository.

## Installing miniconda

Follow the instructions at the [miniconda installation page](https://docs.conda.io/en/latest/miniconda.html#latest-miniconda-installer-links) to install the **conda** program on your system.

1. Open a terminal. 
1. Change directories (`cd`) to the **FRC2023/** directory created by the git cloning process from the previous step
1. Confirm that this directory has a file called **requirements.txt** in it. 

The **requirements.txt** file defines the Python libraries used in the common development environment.


## Create the **frc** conda environment 

Run:

    conda create -n frc python=3.11

You are asking the **conda** program to create an environment. The name of the environment is "frc" (`-n frc`). The version of Python we will use is Python 3.11 (`python=3.11`).

### IMPORTANT! Activate the **frc** environment before work!

When you open a terminal, the **conda** program puts you in the **base** environment, which does not contain any shared libraries and may not even have a Python interpreter installed. You **MUST** activate the **frc** environment, every session, by running:

    conda activate frc 

To confirm that you've switched to the proper environment, run:

    conda info --envs

This will tell you about all the environments you have installed. The output will look something like:

```bash
# conda environments:
#
base                     /home/lobrien/mambaforge
frc                   *  /home/lobrien/mambaforge/envs/frc
```

The currently activated environment will be marked with an asterisk. In this case, you can see that the **frc** environment is activated. 

## Install libraries using **pip**

Install the libraries used by our team with:

    pip install -r requirements.txt

This program will take a minute or two to run. It will download and install the appropriate Python runtime and tools and all the listed packages. We will undoubtedly decide to include some other packages. When we do so, we'll update **requirements.txt** and save it so we don't struggle with "works on my machine!" errors. (There are _always_ WOMM errors! It's just part of the process.)

**Note**: This involves a lot of very complex behind-the-scenes stuff. Often, "recreating the dev environment" is one of the biggest challenges to getting started in a programming project. Don't be surprised if you need help from a mentor at this stage. Just ask!

**Note**: On Windows Subsystem for Linux (WSL), installing **wpilib-util** fails trying to find **std::span**. Larry reckons this indicates that WSL is using an older **gcc**. See https://github.com/WHEARobotics/FRC2023/issues/2

## Verify the most critical tools

Once you have activated the **frc** environment, you should be able to program our robot! First, though, let's verify the installation went well. In a terminal, switch to the **src/helloworld/** subdirectory (`cd ./src/helloworld/`), activate the **frc** conda environment, and:

### Verify you are running Python 3.11

Run `python --version`. Expected result: `Python 3.11.0`

### Verify that you can run a simple wpilib robot

Switch to the **robot/" subdirectory of **src/helloworld/" and run:

    python hello_robot.py sim

You're asking the Python interpreter to run the program **hello_robot.py** in simulator mode. 

You should see a command-line output similar to:

```bash
(frc) D:\src\lobrien\frc_whea\helloworld\robot>python hello_robot.py sim
09:30:18:845 INFO    : wpilib              : WPILib version 2023.1.1.0
09:30:18:846 INFO    : wpilib              : HAL version 2023.1.1.0
09:30:18:846 INFO    : wpilib              : Running with simulated HAL.
09:30:18:846 INFO    : wpilib              : pyntcore version 2023.1.1.0
09:30:18:846 INFO    : wpilib              : robotpy-apriltag version 2023.1.1.0
09:30:18:847 INFO    : wpilib              : robotpy-halsim-gui version 2023.1.1.0
09:30:18:847 INFO    : wpilib              : robotpy-wpimath version 2023.1.1.0
09:30:18:847 INFO    : wpilib              : robotpy-wpinet version 2023.1.1.0
09:30:18:847 INFO    : wpilib              : robotpy-wpiutil version 2023.1.1.0
09:30:18:850 INFO    : halsim_gui          : WPILib HAL Simulation 2023.1.1.0
HAL Extensions: Attempting to load: halsim_gui
Simulator GUI Initializing.
Simulator GUI Initialized!
HAL Extensions: Successfully loaded extension
09:30:20:210 INFO    : pyfrc.physics       : Physics support successfully enabled
09:30:20:230 INFO    : nt                  : Listening on NT3 port 1735, NT4 port 5810
Not loading CameraServerShared
09:30:20:252 INFO    : pyfrc.physics       : Motor config: 2 CIM motors @ 10.71 gearing with 6.0 diameter wheels
09:30:20:253 INFO    : pyfrc.physics       : - Theoretical: vmax=12.980 ft/s, amax=44.731 ft/s^2, kv=0.925, ka=0.268
09:30:20:254 INFO    : pyfrc.physics       : Robot base: 29.5x38.5 frame, 22.0 wheelbase, 110.0 mass

********** Robot program startup complete **********
Default frc::IterativeRobotBase::DisabledPeriodic() method... Override me!
Default frc::IterativeRobotBase::RobotPeriodic() method... Override me!
```

And a "Robot Simulation" Window to appear. Close the simulation window to end the program.

### Verify that you have OpenCV ("Computer Vision") installed

Switch to the **src/helloworld/apriltags** directory and run:

    python hello_apriltag.py

Expected output: `Yeah, that worked.`

### Verify that you can capture an attached Webcam 

**Note**: You must have a Webcam attached or this will fail! lol

Switch to the **src/helloworld/opencv** directory and run:

    python hello_opencv.py

**Note**: You may be asked for permission to access your Webcam and the program may fail until you grant this. 

Expected output: A window should open with the view from your Webcam. It should contain a crude "targeting overlay" made of some crossed lines and a circle. To exit the program, press 'q' on your keyboard. 

## You should be set

If you've:

1. Installed **conda**
1. Created an **frc** environment based on **requirements.txt** 
1. Activated the **frc** environment
1. Verified:
   1. **wpilib**
   1. OpenCV 
   1. Webcam capture

You should have all the tools ready to begin programming! 
