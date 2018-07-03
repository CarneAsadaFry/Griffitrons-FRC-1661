package org.usfirst.frc.team1661.robot;
/**
 * @TODO
 * Do the following at the start of season:
 * 	Download FRC Update Suite
 * 	Firmware, imaging, etc done in RIO configurer- http://roborio-1661-frc.local (when connected to robot)
 * 		This is usually done in internet explorer, as it is a stable browser.
 * 	Re-image roboRIO- imaging software will come with the update suite
 * 	Check for firmware updates- these will also come with the suite
 * 	Replace the CTRE software if needed- this must be on your computer even if you don't actively use it!!!!!!
 * 		Simply google CTRE and see if there is a new piece of software
 * 	Check for CANTalon firmware updates (C:\Users\Public\Public Documents\FRC\TalonSrx-Application-NEWEST_VERSION)
 * 		The version will be listed on the configurer- compare it to the latest build!
 * 	Make sure the code isn't broken- wpilib changes things every year!
 */

/**
 * Imports
 * Most come from wpilib
 * WPI_TalonSRX (talons) comes from CTRE (Cross the Road Electronics)
 * AHRS (navx) comes from Kauai Labs
 */
import java.util.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.hal.PowerJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * @author Nate Tausik & The Power of Friendship
 * @team   Griffitrons #1661
 * @year   2018 Season
 * 
 * Mecanum Drive with setpoint autonomous
 * Interfaces with custom dashboard adapted from FRCDashboard
 * Designed for Power Up- FRC 2018
 */
public class Robot extends IterativeRobot {
	/**
	 * WPI_TalonSRX are CANTalons that are accepted by the MecanumDrive method.
	 * The Talon IDs match the ones found at RIO configuration (http://roborio-1661-frc.local)
	 * 4 Talons are used for the drive train, 2 for the elevator, 2 for intake, 1 to pivot intake, and 2 for the climber
	 */
	WPI_TalonSRX fLeft = new WPI_TalonSRX(0);
	WPI_TalonSRX bLeft = new WPI_TalonSRX(1);
	WPI_TalonSRX fRight = new WPI_TalonSRX(2);
	WPI_TalonSRX bRight = new WPI_TalonSRX(3);

	WPI_TalonSRX elev1 = new WPI_TalonSRX(4);
	WPI_TalonSRX elev2 = new WPI_TalonSRX(5);
	WPI_TalonSRX intake1 = new WPI_TalonSRX(6);
	WPI_TalonSRX intake2 = new WPI_TalonSRX(7);
	WPI_TalonSRX iRotate = new WPI_TalonSRX(8);
	WPI_TalonSRX climber1 = new WPI_TalonSRX(9);
	WPI_TalonSRX climber2 = new WPI_TalonSRX(10);


	/**
	 * An instance of the driver station is used later to read game data, like match time and scale position.
	 * The doubles initialized here are set from the controller input, and control the movement of the robot.
	 * MecanumDrive takes in the CANTalons, and later the magnitude, angle and rotation desired. 
	 * It tells the motors what to do to achieve desired mag, angle and rotation.
	 * XBoxController accepts the index of the USB port, and can give the status of joysticks and buttons. 
	 */
	DriverStation ds = DriverStation.getInstance();

	double x, y, mag, theta, rotation;
	String gameData;
	MecanumDrive myRobot = new MecanumDrive(fLeft, bLeft, fRight, bRight);

	/**
	 * Because of the high number of actions that must be performed with this robot, we decided to split the controls
	 * between two controllers. cubeController controls the intake, while driveController controls drive, elevator and
	 * climber. driveController will be refered to as P1, and cubeController is P2.
	 */
	XboxController cubeController = new XboxController(0);
	XboxController driveController = new XboxController(1);

	/**
	 * Constants that are used so setpoint units can be easily converted to feet or wheel rotations (see below)
	 * Constants are found by using the ratio between setpoint values and pulses, and then pulses and wheel circumference
	 * Setpoint to degrees is found experimentally by feeding values and using the gyro to get the angle traveled.
	 */
	final double SETPOINT_TO_FEET = 36. / Math.PI;
	final double SETPOINT_TO_ROTATIONS = 18;
	final double SETPOINT_TO_DEGREES = 17. / 40;

	/**
	 * Current limits are used when working with 775 motors, which can overheat if they stall too much.
	 * A current limit is set here, and is later used to stop the motor if it exceeds the specified limit.
	 */
	final double ELEV_CURRENT_LIMIT = 50;
	final double INTAKE_CURRENT_LIMIT = 7;
	final double CLIMB_CURRENT_LIMIT = 50;

	/**
	 * These variables are later used for PID control, which works with the encoders to move the robot smoothly in auto.
	 */
	double P, I, D, setpointCount;

	/**
	 * Encoders are declared here. They accept the DIO ports they connect to on the RIO, whether they are inverted, 
	 * and the encoding type (look at the specs for the encoders used).
	 */
	Encoder fLeftEnc = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
	Encoder bLeftEnc = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);
	Encoder fRightEnc = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
	Encoder bRightEnc = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);

	/**
	 * Digital inputs are switches that have a boolean state which changes when pressed. Also called limit switches.
	 * They are typically used as a mechanical stop for certain things. These stop the elevator from moving too far in a direction.
	 */
	DigitalInput upperLim = new DigitalInput(8);
	DigitalInput lowerLim = new DigitalInput(9);

	/**
	 * PIDControllers are declared here. These will be used to make the robot move smoothly the required distance.
	 */
	PIDController fLeftPID, bLeftPID, fRightPID, bRightPID;


	/**
	 * StartPos and the booleans are all values read from the smartdashboard and the driver station. 
	 * They make up the variables that are later used to decide which auto mode should be run.
	 * StartPos is chosen in the dashboard and is the starting position of the robot. 0 = left, 1 = middle, 2 = right.
	 * Pants is another dashboard option that uses more ambitious auto modes. 
	 * The other booleans are read from the driverstation and tell which side of the field each scale is.
	 */
	int startPos = 0;
	boolean pants, scale1Left, scale2Left, scale3Left;

	/**
	 * The maps are used to map different kP values to different distances. This system is a poor man's PID.
	 * Normally, a single P is used for all distances, and it is smoothed with I and D.
	 * If short on time, different P values can be used for each distance. 
	 * With this system, larger P's should be used with smaller distances.
	 */
	Map<Double, Double> drivekP = new HashMap<>();
	Map<Double, Double> rotatekP = new HashMap<>();

	/**
	 * The gyro is used to get the orientation of the robot, and some other non-critical data like temperature.
	 */
	AHRS gyro = new AHRS(I2C.Port.kMXP);

	/**
	 * Robot Initialization- Runs when the robot is started
	 */
	@Override
	public void robotInit() {
		/**
		 * Encoders and gyro are zeroed when the robot starts. This prevents mixing between matches.
		 */
		fLeftEnc.reset();
		bLeftEnc.reset();
		fRightEnc.reset();
		bRightEnc.reset();
		gyro.reset();

		/**
		 * Encoders are set based on wheel diameter and cycles per revolution value so that pulses are converted to inches.
		 */
		fLeftEnc.setDistancePerPulse(6 * Math.PI / 360);
		bLeftEnc.setDistancePerPulse(6 * Math.PI / 360);
		fRightEnc.setDistancePerPulse(6 * Math.PI / 360);
		bRightEnc.setDistancePerPulse(6 * Math.PI / 360);

		/**
		 * This is the section where dashboard variables are initialized.
		 * It is NOT the section where updated values are continuously sent.
		 * All data that will eventually be sent OR received between robot and dashboard must be referenced here.
		 * Initial PID values are placed. 
		 */
		SmartDashboard.putNumber("p", 0.012);
		SmartDashboard.putNumber("i", 0.00);
		SmartDashboard.putNumber("d", 0.00);

		/**
		 * An initial gyro reading is placed. 
		 * gyroReset is a boolean that switches when a button on the dashboard is pressed.
		 */
		SmartDashboard.putNumber("gyro", 0.00);
		SmartDashboard.putBoolean("gyroReset", false);

		/**
		 * Initial encoder values are placed. Another 'button' is created to reset encoders.
		 */
		SmartDashboard.putNumber("flEnc", 0);
		SmartDashboard.putNumber("blEnc", 0);
		SmartDashboard.putNumber("frEnc", 0);
		SmartDashboard.putNumber("brEnc", 0);
		SmartDashboard.putBoolean("encReset", false);

		/**
		 * These values represent the power percentage of motors. It sends 100 when the motor goes forward at full power.
		 * It sends -100 when going backwards at full power.
		 */
		SmartDashboard.putNumber("flDrive", 0);
		SmartDashboard.putNumber("frDrive", 0);
		SmartDashboard.putNumber("blDrive", 0);
		SmartDashboard.putNumber("brDrive", 0);
		SmartDashboard.putNumber("intake", 0);
		SmartDashboard.putNumber("intakerotate", 0);
		SmartDashboard.putNumber("elevator", 0);
		SmartDashboard.putNumber("climber", 0);

		/**
		 * 135 seconds is the total match time, so the timer variable is initialized to this.
		 * We also use the driverstation to tell the dashboard if autonomous is taking place.
		 */
		SmartDashboard.putNumber("timer", 135);
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		/**
		 * These variables keep track of the total voltage and current of the various motors.
		 * This can be useful to tell if a motor is stalling.
		 */
		SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
		SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + bLeft.getOutputCurrent() 
												+ fRight.getOutputCurrent() + bRight.getOutputCurrent());
		SmartDashboard.putNumber("intakedraw", intake1.getOutputCurrent() + intake2.getOutputCurrent());
		SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		SmartDashboard.putNumber("elevatordraw", elev1.getOutputCurrent() + elev2.getOutputCurrent());
		SmartDashboard.putNumber("climberdraw", climber1.getOutputCurrent() + climber2.getOutputCurrent());

		/**
		 * These are mostly here for fun. They are read from the gyro and are kind of cool.
		 */
		SmartDashboard.putNumber("velocity", Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY()));
		SmartDashboard.putNumber("acceleration", Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX()  + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY()));
		SmartDashboard.putNumber("temperature", gyro.getTempC());

		/**
		 * These variables give the smart dashboard the status of the scales. 
		 * It tells which alliance we are on, and the sides of each scale.
		 */
		SmartDashboard.putBoolean("isred", ds.getAlliance().equals(Alliance.Red));
		SmartDashboard.putBoolean("scale1left", false);
		SmartDashboard.putBoolean("scale2left", false);
		SmartDashboard.putBoolean("scale3left", false);

		/**
		 * Automode is a variable that is ultimately read from the dashboard to the robot, 
		 * but it still must be initialized here. It tells the robot which starting position the driver selected 
		 * on the dashboard. Pants is similar, and describes if fancy auto was selected on the dashboard.
		 */
		SmartDashboard.putNumber("automode", 0);
		SmartDashboard.putBoolean("pants", false);

		/**
		 * Initialize PIDControllers with 0 for P, I & D, and encoders and talons for input and output.
		 * Each PIDController turns the wheels specified distances in auto, using encoder data as feedback.
		 * While the values are 0, the Controllers will do nothing.
		 */
		fLeftPID = new PIDController(0, 0, 0, fLeftEnc, fLeft);
		bLeftPID = new PIDController(0, 0, 0, bLeftEnc, bLeft);
		fRightPID = new PIDController(0, 0, 0, fRightEnc, fRight);
		bRightPID = new PIDController(0, 0, 0, bRightEnc, bRight);

		/**
		 * Map certain rotations and distances to kP values for autonomous. These values are experimentally calculated.
		 * As stated above, simply tuning I and D, and using a single P is preferable.
		 */
		drivekP.put(1.5, .012); // For example, this means a P value of .012 should be used for a distance of 1.5 feet.
		drivekP.put(3.5, .012);
		drivekP.put(4.0, .012);
		drivekP.put(4.5, .012);
		drivekP.put(5.0, .012);
		drivekP.put(7.5, .012);
		drivekP.put(11.0, .012);
		drivekP.put(14.0, .012);
		drivekP.put(15.0, .010);
		drivekP.put(22.0, .006);
		drivekP.put(25.0, .006);
		drivekP.put(27.0, .006);

		/**
		 * For rotation, a quadratic regression [f(d) = .0412 - .0003166667 * d + .00000077778 * d * d] was created for 
		 * finding P values for rotation. f(d) = a correct P value where d is degrees.
		 */
		rotatekP.put(45.0, .0285); //This means a P value of .0285 should be used when turning 45 degrees.
		rotatekP.put(90.0, .027);
		rotatekP.put(180.0, .0094);

		/**
		 * The closer loop ramp rate describes the time it should take for a motor to reach the desired speed.
		 * This can be used to smooth the movement of a motor, particularly a powerful one like a 775.
		 */
		elev1.configClosedloopRamp(1.5, 0);
		elev2.configClosedloopRamp(1.5, 0);

		climber1.configClosedloopRamp(25, 0);
		climber2.configClosedloopRamp(25, 0);
	}

	/**
	 * Runs repeatedly while the robot is running.
	 * Mainly used to update data for the SmartDashboard.
	 */
	@Override
	public void robotPeriodic() {
		/**
		 * In this section, the variables that were initialized in robotInit are continuously updated and read from.
		 * This if statement checks if the gyroReset button was clicked on the dashboard, and reset the gyro if so.
		 */
		if(SmartDashboard.getBoolean("gyroReset", false)) {
			gyro.reset();
			SmartDashboard.putBoolean("gyroReset", false);
		}

		/**
		 * This sends the current gyro angle to the dashboard.
		 */
		SmartDashboard.putNumber("gyro", gyro.getAngle());

		/**
		 * This checks if the encoder reset button was pressed, and reset encoders if so.
		 */
		if(SmartDashboard.getBoolean("encReset", false)) {
			fLeftEnc.reset();
			bLeftEnc.reset();
			fRightEnc.reset();
			bRightEnc.reset();
			SmartDashboard.putBoolean("encReset", false);
		}

		/**
		 * This sends the current distance (converted from inches to feet) that the encoders have turned.
		 */
		SmartDashboard.putNumber("flEnc", fLeftEnc.getDistance() / 12);
		SmartDashboard.putNumber("blEnc", bLeftEnc.getDistance() / 12);
		SmartDashboard.putNumber("frEnc", fRightEnc.getDistance() / 12);
		SmartDashboard.putNumber("brEnc", bRightEnc.getDistance() / 12);

		/**
		 * This sends the current power of each drive motor to the dashboard. 
		 * Because of a weird bug that inverts two motors between auto and teleop, 
		 * two values are inverted depending on whether it is auto.
		 */
		if(ds.isAutonomous()) {
			SmartDashboard.putNumber("flDrive", fLeft.get() * 100);
			SmartDashboard.putNumber("frDrive", fRight.get() * 100);
			SmartDashboard.putNumber("blDrive", bLeft.get() * 100);
			SmartDashboard.putNumber("brDrive", bRight.get() * 100);
		}else {
			SmartDashboard.putNumber("flDrive", fLeft.get() * 100);
			SmartDashboard.putNumber("frDrive", -fRight.get() * 100);
			SmartDashboard.putNumber("blDrive", bLeft.get() * 100);
			SmartDashboard.putNumber("brDrive", -bRight.get() * 100); 
		}
		/**
		 * The rest of the motor values are sent to the dashboard only one of each type is sent for the simplicity
		 * of the dashboard.
		 */
		SmartDashboard.putNumber("intake", intake1.get() * 100);
		SmartDashboard.putNumber("intakerotate", iRotate.get() * 100);
		SmartDashboard.putNumber("elevator", elev1.get() * 100);
		SmartDashboard.putNumber("climber", climber1.get() * 100);

		/**
		 * The current match time and auto status are sent to the dashboard for the countdown timer.
		 */
		SmartDashboard.putNumber("timer", ds.getMatchTime());
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		/**
		 * The current voltage and current of each motor is sent to the dashboard continuously.
		 */
		SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
		SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + bLeft.getOutputCurrent() 
												+ fRight.getOutputCurrent() + bRight.getOutputCurrent());
		SmartDashboard.putNumber("intakedraw", intake1.getOutputCurrent() + intake2.getOutputCurrent());
		SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		SmartDashboard.putNumber("elevatordraw", elev1.getOutputCurrent() + elev2.getOutputCurrent());
		SmartDashboard.putNumber("climberdraw", climber1.getOutputCurrent() + climber2.getOutputCurrent());

		/**
		 * These bonus data pieces are also continuously updated on the smart dashboard.
		 */
		SmartDashboard.putNumber("velocity", Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY()));
		SmartDashboard.putNumber("acceleration", Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX()  + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY()));
		SmartDashboard.putNumber("temperature", gyro.getTempC());

		/**
		 * While not strictly necessary, our current alliance (red or blue) is continuously sent to the dashboard.
		 */
		SmartDashboard.putBoolean("isred", ds.getAlliance().equals(Alliance.Red));
	}

	/**
	 * Autonomous Initialization- Runs when auto starts
	 */
	@Override
	public void autonomousInit() {
		/**
		 * Method to receive scale configuration at the start of the match.
		 * Data is given in the form of a three character string (X1)(X2)(X3), where Xi is the ith closest scale 
		 * to your driver station, and X is L or R if you can place cubes on the left or right, respectively.
		 * Examples: LLR, RRL, LLL, RRR, RLR, etc.
		 * The data is taken from the driverstation, and extracted into three booleans which are sent to the dashboard.
		 */
		gameData = ds.getGameSpecificMessage();
		if(gameData.length() >= 3) {
			scale1Left = gameData.charAt(0) == 'L';
			scale2Left = gameData.charAt(1) == 'L';
			scale3Left = gameData.charAt(2) == 'L';
		}

		/**
		 * Gyro is reset.
		 * Some motors run in reverse when given positive setpoints. Those are inverted for autonomous.
		 */
		gyro.reset();
		fRight.setInverted(true);
		bRight.setInverted(true);

		/**
		 * Reads PID values from dashboard and initializes the number of steps for a setpoint.
		 * Setpoint steps are optional but highly recommended. The exact count is up for debate.
		 * They are described in more detail later, but basically smooth auto movement.
		 */
		P = SmartDashboard.getNumber("p", 0);
		I = SmartDashboard.getNumber("i", 0);
		D = SmartDashboard.getNumber("d", 0);
		setpointCount = 75;

		/**
		 * Some of the previously created dashboard variables are read from the dashboard and put into variables.
		 * After their name, they are given default values that are used if no value is found.
		 * These should all be set by the driver on the dashboard. They tell the robot what auto mode to run.
		 */
		startPos = (int)SmartDashboard.getNumber("automode", 1);
		pants = SmartDashboard.getBoolean("pants", false);
		SmartDashboard.putBoolean("scale1left", scale1Left);
		SmartDashboard.putBoolean("scale2left", scale2Left);
		SmartDashboard.putBoolean("scale3left", scale3Left);

		/**
		 * Enables PIDControllers for autonomous. This means they will now control the wheel movement.
		 */
		fLeftPID.enable();
		bLeftPID.enable();
		fRightPID.enable();
		bRightPID.enable();

		/**
		 * Selects auto mode based on starting position, whether PANTS is enabled, and which side the scales are on.
		 * All of these methods are defined at the bottom of the code. They are all used repeatedly.
		 * Auto is put inside of a separate thread. A new thread runs in parallel with the rest of the code.
		 * The thread is used because the thread is slept between commands. If it was not in a separate thread,
		 * the game field would think the code is crashing and would stop the robot.
		 */
		new Thread(() -> {
			/**
			 * The intake is always lowered when the match starts.
			 */
			lowerIntake();
			/**
			 * Describes the auto possibilities when the robot starts to the left.
			 */
			if(startPos == 0) {
				/**
				 * If pants is enabled, the auto selection will prioritize the switch over the scale.
				 * This is used in case our center teammates cannot reliably place a cube, so we can still get the 
				 * auto quest ranking point.
				 */
				if(pants) {
					/**
					 * Places on the left switch. Since this is pants mode, it checks for switch availability first.
					 * It drives forwards rotates and fires. It does not need to raise the elevator all the way
					 * or pivot the intake since the switch is close to the ground.
					 */
					if(scale1Left) {
						autoDrive(14);
						partialRaiseElev();
						autoRotate(90);
						autoDrive(4);
						fire();
						/**
						 * Places on the left scale. Since this is pants, it will only do this if the switch is 
						 * unavailable. It performs a similar operation, but the intake must be fully raised and
						 * pivoted because of the scale's height.
						 */
					}else if(scale2Left) {
						autoDrive(25);
						raiseElev();
						autoRotate(45);
						aim();
						autoDrive(5);
						fire();
						/**
						 * If neither the scale nor the switch is available, the robot will cross the baseline.
						 */
					}else {
						autoDrive(14);
					}
				}else {
					/**
					 * Without pants, auto functions nearly identically. The only difference is that it will check
					 * the availability of the scale first, meaning it will be prioritized over the switch.
					 * This should be used if your middle teammate is confident in their cube placement.
					 */
					if(scale2Left) {
						autoDrive(25);
						raiseElev();
						autoRotate(45);
						aim();
						autoDrive(5);
						fire();
					}else if(scale1Left) {
						autoDrive(14);
						partialRaiseElev();
						autoRotate(90);
						autoDrive(4);
						fire();
					}else {
						autoDrive(14);
					}
				}
			}else if(startPos == 1) {
				/**
				 * For middle auto, the robot will only attempt to place on the switch. It will find the side of
				 * the switch that is controller by our alliance, and it will place there. 
				 */
				if(scale1Left) {
					autoDrive(5);
					autoRotate(-90);
					autoDrive(5.0);
					partialRaiseElev();
					autoRotate(90);
					autoDrive(7.5);
					fire();
				}else {
					autoDrive(5);
					autoRotate(90);
					autoDrive(5.0);
					partialRaiseElev();
					autoRotate(-90);
					autoDrive(7.5);
					fire();
				}
			}else {
				/**
				 * The right auto works identically to the left. The only difference is that the rotations are reversed,
				 * as the robot will be approaching the scale or switch from the right. Duh.
				 */
				if(pants) {
					if(!scale1Left) {
						autoDrive(14);
						partialRaiseElev();
						autoRotate(-90);
						autoDrive(4);
						fire();
					}
					else if(!scale2Left) {
						autoDrive(25);
						raiseElev();
						autoRotate(-45);
						aim();
						autoDrive(5);
						fire();
					}else {
						autoDrive(14);
					}
				}else {
					if(!scale2Left) {
						autoDrive(25);
						raiseElev();
						autoRotate(-45);
						aim();
						autoDrive(5);
						fire();
					}else if(!scale1Left) {
						autoDrive(14);
						partialRaiseElev();
						autoRotate(-90);
						autoDrive(4);
						fire();
					}else {
						autoDrive(14);
					}
				}
			}
		}).start();
	}

	/**
	 * Runs repeatedly during autonomous. Can describe movement.
	 * Autonomous periodic is empty for us because we describe auto movements as commands that happen a single time.
	 * If auto instead relied on a timer (please future programmer just use encoders) it would be run from here,
	 * since the movement of the robot will need to be updated based on time.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * Teleop Initialization- Runs when driver controlled period starts
	 * Teleop init is typically used to update any parameters that need to be changed between teleop and autonomous.
	 */
	@Override
	public void teleopInit() {
		/**
		 * PID is disabled so it will not interfere with driver control.
		 */
		fLeftPID.disable();
		bLeftPID.disable();
		fRightPID.disable();
		bRightPID. disable();

		/**
		 * Motors that had to be inverted for auto are un-inverted.
		 * It is worth noting that I am still unaware as to why some motors act like this.
		 * If you can figure it out, get yourself a cookie.
		 */
		fRight.setInverted(false);
		bRight.setInverted(false);
	}

	/**
	 * Teleop Periodic- Driver controls the robot
	 * Teleop periodic is a continuously updating method during the driver controlled period. Use it to control the  
	 * robot, since the driver's input must be continuously updated.
	 */
	@Override
	public void teleopPeriodic() {
		/**
		 * Our main goal is to translate the xy coordinates from the XBox controller and turn it into polar coordinates.
		 * rawaxis0 is left-right on the left stick, and rawaxis1 is the up-down
		 * The distance between two points formula is used to find the magnitude of the movement.
		 * The movement is ignored if the magnitude is below 25% of maximum, since the sticks do not rest perfectly at 0.
		 * At the time of writing this code, the y axis is inexplicably inverted on XBox controllers, hence the (-).
		 */
		x = driveController.getRawAxis(0);
		y = -driveController.getRawAxis(1);
		if(Math.sqrt((x * x) + (y * y)) > 0.25)
			mag = Math.sqrt((x * x) + (y * y));
		else
			mag = 0;

		/**
		 * rawaxis4 is the left-right axis of the right stick. It controls rotation, and has a deadzone for the same
		 * reason as above.
		 */
		if(Math.abs(driveController.getRawAxis(4)) > 0.25)
			rotation = driveController.getRawAxis(4);
		else
			rotation = 0;

		/**
		 * The angle at which the robot should drive is determined here using trigonometry. 
		 * Note that radians must be converted to degrees for use with the mecanumdrive class.
		 */
		if(y < 0){
			if(x > 0)
				theta = 180 - Math.abs(Math.atan(x/y) * 180 / Math.PI);
			else
				theta = 180 + Math.atan(x/y) * 180 / Math.PI;
		}else{
			if(x > 0)
				theta = Math.atan(x/y) * 180 / Math.PI;
			else
				theta = Math.atan(x/y) * 180 / Math.PI + 360;
		}

		/**
		 * Locks drive to cardinal directions, unless right bumper is held.
		 * Makes it easier to go straight, and precise angles are usually unnecessary.
		 */
		if(!driveController.getBumper(Hand.kRight)) {
			if(theta > -45 && theta <= 45)
				theta = 0;
			else if(theta > 45 && theta <= 135)
				theta = 90;
			else if(theta > 135 && theta <= 225)
				theta = 180;
			else
				theta = 270;
		}

		/**
		 * Slows drive to 30% and rotation to 20% when left bumper is held down. Used for precise movement.
		 * Note that rotation is locked to 75% of its maximum power by default. We found that rotation was 
		 * too fast to precisely use at full speed.
		 */
		if(driveController.getBumper(Hand.kLeft))
			myRobot.drivePolar(mag * 0.3, theta, rotation * 0.2);
		else
			myRobot.drivePolar(mag, theta, rotation * 0.75);

		/**
		 * P2's triggers control the intake. Right for in, left for out. 
		 * The speed mirrors the amount the trigger is pressed.
		 * The intake can also be controlled using P2's A and B buttons, which make it move at full power.
		 * 775s have a tendency to overheat when stalled, so a current limit is used to prevent this from happening.
		 * A small deadzone is added as usual.
		 * When the intake is not being moved, the motors very slightly stall inward to keep the cube in place.
		 */
		if(cubeController.getTriggerAxis(Hand.kRight) > 0.2 && intake1.getOutputCurrent() < INTAKE_CURRENT_LIMIT
				&& intake2.getOutputCurrent() < INTAKE_CURRENT_LIMIT) {
			intake1.set(cubeController.getTriggerAxis(Hand.kRight));
			intake2.set(-cubeController.getTriggerAxis(Hand.kRight));
		}else if(cubeController.getTriggerAxis(Hand.kLeft) > 0.2 && intake1.getOutputCurrent() < INTAKE_CURRENT_LIMIT
				&& intake2.getOutputCurrent() < INTAKE_CURRENT_LIMIT) {
			intake1.set(-cubeController.getTriggerAxis(Hand.kLeft));
			intake2.set(cubeController.getTriggerAxis(Hand.kLeft));
		}else if(cubeController.getAButton() && intake1.getOutputCurrent() < INTAKE_CURRENT_LIMIT
				&& intake2.getOutputCurrent() < INTAKE_CURRENT_LIMIT) {
			intake1.set(1.00);
			intake2.set(-1.00);
		}else if(cubeController.getBButton() && intake1.getOutputCurrent() < INTAKE_CURRENT_LIMIT
				&& intake2.getOutputCurrent() < INTAKE_CURRENT_LIMIT) {
			intake1.set(-1.00);
			intake2.set(1.00);
		}else {
			intake1.set(0.0);
			intake2.set(0.0);
		}
		if(driveController.getTriggerAxis(Hand.kRight) > 0.2 && upperLim.get() 
				&& elev1.getOutputCurrent() < ELEV_CURRENT_LIMIT && elev2.getOutputCurrent() < ELEV_CURRENT_LIMIT) {
			elev1.set(-driveController.getTriggerAxis(Hand.kRight));
			elev2.set(-driveController.getTriggerAxis(Hand.kRight));
		}else if(driveController.getTriggerAxis(Hand.kLeft) > 0.2 && lowerLim.get()
				&& elev1.getOutputCurrent() < ELEV_CURRENT_LIMIT && elev2.getOutputCurrent() < ELEV_CURRENT_LIMIT) {
			elev1.set(driveController.getTriggerAxis(Hand.kLeft) * 0.8);
			elev2.set(driveController.getTriggerAxis(Hand.kLeft) * 0.8);
		}else {
			elev1.set(-0.05);
			elev2.set(-0.05);
		}

		/**
		 * Activates climber. Can only be turned on if in the last 30 seconds of the match to prevent accidental usage.
		 * Like with the intake motors, a current limit is placed to prevent the motors from overheating.
		 */
		if(driveController.getAButton() && ds.getMatchTime() <= 30 
				&& climber1.getOutputCurrent() <= CLIMB_CURRENT_LIMIT 
				&& climber2.getOutputCurrent() <= CLIMB_CURRENT_LIMIT) {
			climber1.set(-1.0);
			climber2.set(-1.0);
			driveController.setRumble(RumbleType.kLeftRumble, 1.0);
			driveController.setRumble(RumbleType.kRightRumble, 1.0);
			cubeController.setRumble(RumbleType.kLeftRumble, 1.0);
			cubeController.setRumble(RumbleType.kRightRumble, 1.0);
		}else {
			climber1.set(0);
			climber2.set(0);
			driveController.setRumble(RumbleType.kLeftRumble, 0);
			driveController.setRumble(RumbleType.kRightRumble, 0);
			cubeController.setRumble(RumbleType.kLeftRumble, 0);
			cubeController.setRumble(RumbleType.kRightRumble, 0);
		}

		/**
		 * Pivots the intake mechanism using the D Pad.
		 * Like with the intake, the pivot motor stalls slightly by default to keep the intake in place.
		 * This sketchy solution can be replaced with PID if the time is available.
		 */
		if(cubeController.getPOV(0) != -1 && (cubeController.getPOV(0) >= 315 || cubeController.getPOV(0) <= 45))
			iRotate.set(.5); 
		else if(cubeController.getPOV(0) != -1 && cubeController.getPOV(0) >= 135 && cubeController.getPOV(0) <= 225)
			iRotate.set(-.5);
		else
			iRotate.set(0.05);
	}

	/**
	 * Test Initialization- Runs when test mode is started.
	 * Test mode can be turned on from the driver station. Test things here that should happen once that should not
	 * interfere with the main code.
	 */
	@Override
	public void testInit() {

	}

	/**
	 * Used to test various robot functionality.
	 * Things that need to happen repeatedly should be tested here (i.e. controlling a motor)
	 * This method activates after testInit() and starts when Test is pressed on the driver station.
	 */
	@Override
	public void testPeriodic() {

	}

	/**
	 * Disabled Initialization- Runs when the robot is first disabled.
	 * Probably don't use this unless you have some really genius plan. Nothing but trouble.
	 */
	@Override
	public void disabledInit() {

	}

	/**
	 * Runs repeatedly when the robot is disabled.
	 * Same applies as with disabledInit(). Just don't.
	 */
	@Override
	public void disabledPeriodic() {

	}

	/**
	 * Auto drive is called during autonomous mode. It accepts a distance in feet, and makes the robot go that distance.
	 * Negative is backwards, and only forward and backwards are supported here.
	 */
	public void autoDrive(double d) {
		/**
		 * Encoder values are reset so the robot will go the distance of the setpoint from its current location.
		 */
		fLeftEnc.reset();
		bLeftEnc.reset();
		fRightEnc.reset();
		bRightEnc.reset();
		
		/**
		 * All distances that will be used during autonomous must be added to the drivekP map. 
		 * Here, the PID controller's P value is set the value the distance has been mapped to.
		 * The P value currently being used is sent to the smart dashboard.
		 */
		fLeftPID.setP(drivekP.get(Math.abs(d)));
		bLeftPID.setP(drivekP.get(Math.abs(d)));
		fRightPID.setP(drivekP.get(Math.abs(d)));
		bRightPID.setP(drivekP.get(Math.abs(d)));
		SmartDashboard.putNumber("p", drivekP.get(Math.abs(d)));

		/**
		 * For a smoother autonomous movement, the setpoint is split into smaller setpoints.
		 * Each one is run for 25 ms before the next one starts. By letting the robot go a small disance and then
		 * go another small disance, it goes much straighter. The SETPOINT_TO_FEET constant is used to the value fed
		 * to the method will actually make it go that many feet. 
		 * SetpointCount is a somewhat arbitrary variable defined further up which was messed with to find a setting
		 * that was both smooth and relatively quick. It is the number of subdivisions the setpoint becomes.
		 * 
		 * SIDENOTE: The total time an autonomous move will take can be found by multiplying the setpoint count
		 * by the sleep time and then adding the 500 ms delay at the end. Convert to seconds by / 1000.
		 * For example, each forward or backward movement should take (25 * 25 + 500) / 1000 = 1.125 seconds!
		 */
		for (int i = 1; i <= setpointCount; i++) {
			fLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
			bLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
			fRightPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
			bRightPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
			try {
				Thread.sleep(25);
			}catch(InterruptedException e) {}
		}
		
		/**
		 * At the end, the controller is told to move towards the overall setpoint, and is given 500 ms to do so.
		 * This is to make sure it accurately reaches the destination before moving on to the next command.
		 * SIDENOTE: The 500 ms time is most likely much more time than needed. Try lowering this to save time in auto.
		 */
		fLeftPID.setSetpoint(d * SETPOINT_TO_FEET);
		bLeftPID.setSetpoint(d * SETPOINT_TO_FEET);
		fRightPID.setSetpoint(d * SETPOINT_TO_FEET);
		bRightPID.setSetpoint(d * SETPOINT_TO_FEET);
		try {
			Thread.sleep(500);
		}catch(InterruptedException e) {}
	}

	/**
	 * Used during autonomous to make the robot rotate.
	 * d is the angle in degrees that the robot should turn during the command.
	 * Positive is clockwise, negative is counterclockwise.
	 */
	public void autoRotate(double d) {
		/**
		 * Encoders are reset so it will rotate entirely from its current position.
		 */
		fLeftEnc.reset();
		bLeftEnc.reset();
		fRightEnc.reset();
		bRightEnc.reset();
		
		/**
		 * Unlike with auto drive, a P value is determined using an experimentally calculated quadratic regression.
		 * The purpose is so the robot will move with more precision when given low setpoints.
		 * Like with autoDrive(), each angle must be mapped before hand in rotatekP so it can be accessed here.
		 * The P value is also updated for the smart dashboard.
		 */
		fLeftPID.setP(rotatekP.get(Math.abs(d)));
		bLeftPID.setP(rotatekP.get(Math.abs(d)));
		fRightPID.setP(rotatekP.get(Math.abs(d)));
		bRightPID.setP(rotatekP.get(Math.abs(d)));
		SmartDashboard.putNumber("p", rotatekP.get(Math.abs(d)));

		/**
		 * This setpoint is also split into smaller segments. Note that the right motors spin backwards.
		 * By doing so, the robot will rotate clockwise. If d is negative, the left motors spin backwards instead,
		 * causing the robot to turn counterclockwise.
		 */
		for(int i = 1; i <= setpointCount; i++) {				
			fLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_DEGREES);
			bLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_DEGREES);
			fRightPID.setSetpoint(((i * -d) / setpointCount) * SETPOINT_TO_DEGREES);
			bRightPID.setSetpoint(((i * -d) / setpointCount) * SETPOINT_TO_DEGREES);
			try{
				Thread.sleep(25);
			} catch(InterruptedException e) {}
		}
		fLeftPID.setSetpoint(d * SETPOINT_TO_DEGREES);
		bLeftPID.setSetpoint(d * SETPOINT_TO_DEGREES);
		fRightPID.setSetpoint(-d * SETPOINT_TO_DEGREES);
		bRightPID.setSetpoint(-d * SETPOINT_TO_DEGREES);
		try{
			Thread.sleep(500);
		} catch(InterruptedException e) {}
	}

	/**
	 * This method is called during autonomous, and raises the elevator to about its maximum height.
	 * The method's contents are wrapped in a thread so this action will happen while the robot continues to drive
	 * or rotate. This saves large amounts of time during autonomous. 
	 */
	public void raiseElev() {
		new Thread(() -> {
			/**
			 * Despite the inclusion of a limit switch, a timer is used to make sure that the elevator does not 
			 * continuously stall at the top. Knowing this team, adding electronic stops to compliment mechanical
			 * ones is highly recommended whenever possible.
			 */
			Timer time = new Timer();
			time.start();
			time.reset();
			
			/**
			 * The elevator will continue to climb as long as the limit switch has not been hit and the timer has not
			 * been running for 4.5 seconds or more. 
			 */
			while(upperLim.get() && time.get() < 4.5) {
				/**
				 * This current limit is once again checked to ensure the elevator does not stall and our motors do not
				 * break. If this happens, they will immediately stop. 
				 */
				if(elev1.getOutputCurrent() < ELEV_CURRENT_LIMIT && elev2.getOutputCurrent() < ELEV_CURRENT_LIMIT) {
					elev1.set(-0.4);
					elev2.set(-0.4);
				}else {
					elev1.set(0.0);
					elev2.set(0.0);
				}
			}
			
			/**
			 * After the elevator has reached the top, it will slightly stall to prevent it from falling back down.
			 * This stall is not enough to damage the motors, but will keep the elevator in place.
			 */
			elev1.set(-0.05);
			elev2.set(-0.05);
			time.stop();
		}).start();
	}

	/**
	 * Partial raise elevator functions almost identically to raise elevator, but it is only active for 1.5 seconds.
	 * The purpose of this method is to save time when placing on the switch in autonomous, as there is no need to
	 * raise the elevator fully.
	 * It is also in a thread so it can act parallel with driving.
	 */
	public void partialRaiseElev() {
		new Thread(() -> {
			Timer time = new Timer();
			time.start();
			time.reset();
			while(upperLim.get() && time.get() < 1.5) {
				if(elev1.getOutputCurrent() < ELEV_CURRENT_LIMIT && elev2.getOutputCurrent() < ELEV_CURRENT_LIMIT) {
					elev1.set(-0.4);
					elev2.set(-0.4);
				}else {
					elev1.set(0.0);
					elev2.set(0.0);
				}
			}
			elev1.set(-0.05);
			elev2.set(-0.05);
			time.stop();
		}).start();
	}

	/**
	 * This method can be called during autonomous and once again functions similarly to the other elevator methods.
	 * Notable differences are that the elevator now moves down instead of up, it moves slightly faster, and it moves
	 * for significantly less time.
	 * It is also in a thread so it can act parallel with driving. 
	 * SIDENOTE: This method was not used during the season. This means it is mostly untested. In reflection of some
	 * problems during the season, it would probably be wise to lower the speed and raise the time allotted for 
	 * the lowering process.
	 */
	public void lowerElev() {
		new Thread(() -> {
			Timer time = new Timer();
			time.start();
			time.reset();
			while(lowerLim.get() && time.get() < 1.9) {
				if(elev1.getOutputCurrent() < ELEV_CURRENT_LIMIT && elev2.getOutputCurrent() < ELEV_CURRENT_LIMIT) {
					elev1.set(0.6);
					elev2.set(0.6);
				}else {
					elev1.set(0.0);
					elev2.set(0.0);
				}
			}
			elev1.set(-0.05);
			elev2.set(-0.05);
			time.stop();
		}).start();
	}

	/**
	 * This method is called during autonomous, and lowers the intake at the start of the match.
	 * It functions similarly to the elevator methods with a few key differences. 
	 * There is no fear of breaking the intake when lowering it, so there it relies solely on the timer to stop it.
	 * It also does not need a current limit, as simply lowering the intake has no chance of stalling. 
	 * Like with many other methods, it uses a thread so it can run parallel to driving, and stalls slightly to hold
	 * its position.
	 */
	public void lowerIntake() {
		new Thread(() -> {
			Timer time = new Timer();
			time.start();
			time.reset();
			while(time.get() < .8) {
				iRotate.set(-0.5);
			}
			iRotate.set(0.05);
			time.stop();
		}).start();
	}

	/**
	 * The aim method brings the intake back up half way. This is done so a cube can be placed on the scale in 
	 * autonomous, even if the scale was already taken by the other team. It gives a small amount of extra height.
	 * The method raises the intake for 1 second, which was experimentally calculated to be around the time required
	 * to bring the mechanism half way. Afterwards, it is slightly stalled to hold its position.
	 * Like many other methods, it is placed in its own thread so it will occur in parallel with driving.
	 */
	public void aim() {
		new Thread(() -> {
			Timer time = new Timer();
			time.start();
			time.reset();
			while(time.get() < 1.0) {
				iRotate.set(0.5);
			}
			iRotate.set(0.05);
			time.stop();
		}).start();
	}

	/**
	 * The fire method is called during autonomous and is used to fire a cube onto the switch or scale.
	 * It works similarly to all other autonomous mechanism control.
	 * The movement of the intake is the safest of all as it can move indefinitely without reaching an end or breaking.
	 * As a result, and because this is always the last autonomous action, the timer goes on for much longer than it
	 * has to. A current limit is still used in case a cube gets stuck and the motors begin to stall. It will prevent
	 * the possible destruction of the motors. The intake is not stalled at the end because why would you?
	 */
	public void fire() {
		Timer time = new Timer();
		time.start();
		time.reset();
		while(time.get() < 1.5) {
			if(intake1.getOutputCurrent() < INTAKE_CURRENT_LIMIT && intake2.getOutputCurrent() < INTAKE_CURRENT_LIMIT) {
				intake1.set(-1.0);
				intake2.set(1.0);
			}else {
				intake1.set(0.0);
				intake2.set(0.0);
			}
		}
		intake1.set(0.0);
		intake2.set(0.0);
	}
}
