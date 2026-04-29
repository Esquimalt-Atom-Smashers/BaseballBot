# Welcome
This is the Repository for **FRC 2026 game, REBUILT** for the Esquimalt Robotics Teams (**7287 Esquimalt Atom Smashers** and **7334 Esquimalt Proton Pulverizers**) from Esquimalt, BC, Canada.

View a description of our autos [here](/AUTOS.md/)

# Setup
To setup this Repository, please do the following:
1. Ensure you have the correct seasons's [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html) installed. (Check this by opening the tools and ensure they all say 2026, or in the Driver Station's case, is V26 or higher.)
2. Clone the Repository to your computer
3. Open AdvantageScope, then navigate to the [AdvantageScope Folder](/AdvantageScope/) and follow the [README](/AdvantageScope/README.md) instructions to setup AdvantageScope.
4. Open the Driver Station, then navigate to the "Setup" tab (3rd tab on the left,) and add the correct Team Number, and set the "Dashboard Type" to "Elastic."
5. Elastic now should be open. If not, open Elastic, then navigate to the [layout folder](/layout/) and follow the [README](/layout/README.md) instructions to setup Elastic.

## Simulation Setup
For setting up the Simulation, please do the following:
1. Follow the Simulation setup instructions for [AdvantageScope](/AdvantageScope/README.md) and [Elastic](/layout/README.md)
2. In the VSCode Command Palette (the search bar on the top, also accessible by pressing **Ctrl + Shift + P** on Windows/Linux or **Shift + Command + P** on Mac,) type in **"WPILib: Simulate Robot Code"**. After it builds, the Command Palette will have a window, click "OK" to start the Simulation.

## Common Errors
Here are some common errors and how to resolve them.

1. <u>After building, it states it can't find lots of files or methods. Please try the below fixes (in order): </u> <br>
    - It is likely the Java Server is simply not Cached correctly. In the VSCode Command Palette (the search bar on the top, also accessible by pressing `Ctrl + Shift + P` on Windows/Linux or `Shift + Command + P` on Mac,) type in `">Java: Clean java Language Server Workspace"`. On the bottom-right of your screen, you will see a pop-up asking **"Are you sure you want to clean the Java language server workspace?"**, select `"Reload and delete"`. VSCode will now reload, and you should be able to build without issues.
    - Open the Visual Studio Code terminal and type `.\gradlew clean` and then `.\gradlew build`.
    - Delete and re-clone the repository from your files. It is possible that some files corrupted/build errors occured overtime, and re-cloning the repo should help it.
    - Restart your computer.
    - If all else fails, you may need to reinstall WPIlib (you may want to delete the repository, and uninstall WPILib, then reinstall WPILib, and then restart your computer before trying again).
2. <u>The Simulation starts becoming laggy after running for a long time.</u> <br>
    - As the Simulation is running a 2D and 3D physics enginer with lots of Assets, the Simulator is likely going to degrade in performance the longer you run it. To resolve this, go back to VSCode, and in the pop-up bar near the top of the window, click the Green Counter-Clockwise arrow (or `Ctrl + Shift + F5` on Windows/Linux or `Shift + Command + F5` on Mac.) The Simulation will reboot, and will return to its original starting speed. 
    - If the starting speed is slow, you can disable Fuel visualization in AdvantageScope by clicking the "X" icon in the bottom window of AdvantageScope for Fuel, and/or only spawning half the Fuel by changing the **"showHalfFuel"** boolean on line *317* of [FuelSim.java](/src/main/java/frc/robot/simulation/FuelSim.java) to **"false"**. 
     - If the simulation is still laggy, in AdvantageScope you can go to `App -> Show Prefrences -> Discard Live Data -> Change to 1 minute`
     - If these dont help, then unfortunately, the only solution is getting a more powerful computer.
3. <u>When running the Simulation, the Driver Station can't find a robot to connect to.</u> <br>
Ensure that after you build the Simulation, that the option **"Use Real DriverStation"** is selected.

For any other issues, please don't hesitate to [Contact Us](https://www.esquimaltatomsmashers.ca/contact).

# Simulator
This project uses Simulator heavily to encourage additional Driver Practice without a robot, as well as testing and developing programs without the Real Robot.

More details to come, but TLDR:
Our simulation allows us to accurately simulate what our robot does and how it behaves in a match, including indepth fuel Physics, such as getting stuck on fuel on walls. We can enable up to 6 robots on the field, each with their own automated dynamic behaviour type, or they can be human controlled instead.

We will update the README soon with more details.

The Robot and Field interact using [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/Maple-Sim), a 2D dyn4j Physics engine to simulate 2D forces and collisions, allowing testing realistic driving and interactions.

The Fuel interacts with both using a [custom Simulation](https://github.com/hammerheads5000/FuelSim) (similar to a 3D Physics engine) to simulate forces such as gravity, air-resistance, etc. You can reset the fuel to its original position by clicking the `Reset Fuel` button in the **"Shooting Test"** tab in elastic. Alternatively, it could be better to do the second fix in the **"Common Errors"** section above.

Both were originally derivied from different teams (linked above), and we thank them for their amazing work.

# Robot Description
The Robot is a Swerve-base robot with a auto-aiming rotating Turret to allow shooting from most places on the field.

# Subsystems
List of Subsystems and what they do, subsytems are listed in order from Fuel interaction, from first intaked, to shooting out of the Robot. The order is as follows: <br>
Intake, Extender, Agitator, Transfer, Turret, Hood, Flywheel.

Each Subsystem is designed to be modular, to not rely on any other Subsytems to allow easy maintenance, debugging, and transfer to other and future projects if desired. Interactions between Subsystems are controlled by other "Manager" classes.


### Note for control types
#### Voltage Control
All Voltage control systems have a Open-loop ramp (limits how fast the system ramps up to target voltage, reducing system stress) and Voltage Compensation, to ensure consistency regardless of Battery Voltage.

### Positional and Velocity Control
Positional control systems use a PIDFVS to get to its target position. These values MUST be tuned to work well.

Tuning is as follows:
1. Set all gains to zero (kP, kI, kD, kV, kS).
2. Increase **kS** until the motor just barely starts to move (static friction).
3. Increase **kV** until measured velocity tracks the setpoint well (velocity feedforward).
4. Increase **kP** until the response starts to oscillate, then back off slightly.
5. Increase **kD** to reduce overshoot/jitter without adding noise.

It is unlikely you will need to modify **kI**.


## Intake
The Intake is responsible for collecting the Fuel off the Field.

It is one motor (SparkMax Neo550) using Voltage Control.

It has four states: IDLE, INTAKING, REVERSING, MANUAL.

## Extender
The Extender is responsible for deploying/retracting the Intake.

It is one motor (Sparkmax Neo550) using Positional Control.

It has five states: IDLE, RETRACTED, PARTIAL, EXTENDED, MANUAL.

## Agitator
The Agitator is responsible for moving Fuel from the Storage area into the Shooter Transfer.

It is one motor (SparkMax Neo550) using Voltage Control.

It has four states: IDLE, STAGING, SHOOTING, MANUAL.

## Shooter Subsystem
The Shooter Subsystem houses the Transfer, Turret, Hood, and Flywheel Subsystems. It also houses the "Shooter Managers".

The Shooter files works together to coordinate between all Systems related to shooting, from the Agitator to the Flywheel, coordinating them to work together to shoot a ball. The Shooter target values (Position, Angle, etc.)

### <u>Transfer</u>
The Transfer is responsible for moving Fuel from the Agitator through the Shooter and into the Flywheel.

It is a one motor (SparkMax Brushed) using Voltage Control.

It has four states: IDLE, STAGING, SHOOTING, MANUAL.

### <u>Turret</u>
The Turret is responsible for rotating the Shooter so that it is aimed at the correct target.

It is a one motor (Sparkmax Neo550) using Positional Control.

It has four states: IDLE, TRACKING, AT_TARGET, MANUAL.

### <u>Hood</u>
The Hood is responsible for changing the Fuel exit angle so that it will reach the correct target.

It is a Axon Max Servo using Positional control - 0 and 90 deg measured in setPosition, and then linear interpolation used to find the set value to go to angles in between.

It has four states: IDLE, TRACKING, AT_TARGET, MANUAL.

### <u>Flywheel</u>
The Flywheel is responsible for ensuring the Flywheel reaches the correct Velocity to shoot the Fuel.

It is a one motor (TalonFX) using Velocity Control.

It has four states: IDLE, CHARGING, AT_SPEED, MANUAL.
