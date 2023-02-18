# Hello, limelight!

This code is not much more advanced than the other "hello, world" programs, but
it requires the RoboRio and Limelight and network all to be up and running.

# Networking Configuration

**NOTE!!**: The Limelight expects the RoboRio's IP address to be **10.38.81.2** and expects it's own IP address to be **10.38.81.11**. I am not sure if these are MUST or SHOULD, but if it's easy: use those.  

1. Start up the RoboRio / Router / Limelight combo. I had difficulty with the network connections to the router: the lights on the e'net ports on the devices should light up and the router has two bright blue LEDs that light when the e'net ports are connected. For me, this only rarely happened.

My workaround: I used my own router. 

2. Once the RoboRio and Limelight are up on a Wifi network, connect to the WiFi network with your driver / development laptop.

3. Open an Anaconda terminal on your machine
   1. Activate the **frc** conda environment
   2. Switch to this directory (**src/helloworld/limelight/**)
4. Run `python robot.py deploy`. You should see some messages ending with "Deploy was successful". If you have trouble, refer to the file [Developing and Deploying](../../DEVELOPING_AND_DEPLOYING.md).
5. Start your FRC Driver Station
6. Confirm that "Robot code" is  green in the FRC Driver Station. If it is red or flickering, troubleshoot using the steps in the [Developing and Deploying](../../DEVELOPING_AND_DEPLOYING.md) document.
7. Open a browser at [http://10.38.81.11:5801/](http://10.38.81.11:5801) and [http://10.38.81.11:5802/](http://10.38.81.11:5802)
8. In Smart Dashboard, find the value "limelight driver pipeline" and note that it is **true** and the feed from the limelight is full exposure. Set the value of "limelight driver pipeline" to **false**. The camera parameters will change, the LEDs will come on, and you can now locate retroreflective tape targets with the limelight. 

The next step after this is switching the limelight pipeline based on a controller input. For instance, if you press 'A' on the XBox Controller, toggle between `self.limelight.putNumber("pipeline", PIPELINE_ID_DRIVER)` and `self.limelight.putNumber("pipeline", PIPELINE_ID_RETROREFLECTIVE)`.

