## [Vision Processing](https://docs.wpilib.org/en/stable/docs/software/vision-processing/introduction/what-is-vision.html)
Vision utilizes one or more cameras attached to the robot to help achieve the robot's goal.  In the simplest form, the camera output can be sent to the driver station as a visual aid for the human driver.  However, the more interesting use of cameras is to perform vision analysis on the camera data to help the robot make decisions.  The vision data could be used to automatically identify the location of targets, field elements, etc.  In more advanced uses, the vision data can be used to help determine the robot's position on the field and update the robot's odometry information.

### PhotonVision
In this example, we are using [PhotonVision](https://docs.photonvision.org/en/latest/index.html) to implement a simple command that causes the robot to track the nearest AprilTag that it sees.  If an AprilTag is not detected, then the robot sits still.

PhotonVision requires a separate coprocessor for it to run on. Fortunately, the Romi's Raspberry Pi has sufficient CPU performance to handle more basic vision processing tasks.

Before continuting, you must follow the [PhotonVision Romi Installation](https://docs.photonvision.org/en/latest/docs/getting-started/installation/sw_install/romi.html) instructions.

Unfortunately, the installation process requires the use of an actual ethernet connection to download the PhotonVision binaries from the internet.  When plugging in the Romi's Raspberry Pi via ethernet, the address will very likely *not* be `10.0.0.2`.  Instead, an IP address will be assigned.  Since the Raspberry Pi doesn't have a display, it's not obvious how to get the IP address.  There are lots of ways to find the IP address. One tool you can use is [Angry IP Scanner](https://angryip.org/) running a PC that is *on the same network* to search for the Raspberry Pi IP address.

If PhotonVision is successfully installed, you should be able to access the GUI at: `http://photonvision.local:5800/`

#### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/896c498868d95fc60e0cd397154a503722345a0c?diff=split):
1. `RobotContainer.java`
    1. Enable the new `DriveVision()` command while the controller's `X` button is pressed
1. `DriveVision.java`
    1. Implement a simple AprilTag tracking algorithm. (Based on the [Aiming at a Target](https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html) example.)
1. `Drivetrain.java`
    1. Create a `stop()` method to explicitly stop the robot's drivetrain motion
1. `vendordeps/photonlib.json`
    * [Install PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html#what-is-photonlib)
