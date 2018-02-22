package org.usfirst.frc.team1661.robot;
/**
 * @TODO- Beginning of season
 * Download FRC Update Suite
 * Firmware, imaging, etc done in RIO configurer- Open Internet Explorer when connected to RIO, go to (http://roborio-1661-frc.local)
 * Re-image roboRIO
 * Check for firmware updates
 * Check for new CTRE software (CTRE Phoenix)
 * Check for CANTalon firmware updates (C:\Users\Public\Public Documents\FRC\TalonSrx-Application-NEWEST_VERSION)
 * Look at javadocs, fix any updates to wpilib
 */

/**
 * Imports
 * All from wpilib except TalonSRX, which come from CTRE package (phoenix)
 */
import java.util.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.hal.PowerJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

/**
 * @author Nathan Tausik, Griffitons #1661 / The Power of Friendship
 * 
 * Encoded Mecanum Drive with setpoint autonomous
 * Interfaces with custom dashboard adapted from MIT's FRCDashboard
 * Designed for FRC 2018- Power Up
 */
public class Robot extends IterativeRobot {
	/**
	 * WPI_TalonSRX are CANTAlons that are accepted by the MecanumDrive method.
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


	/**
	 * Initialization for other misc. variables.
	 * The doubles initialized here are set from the controller input, and control the movement of the robot.
	 * MecanumDrive takes in the CANTalons, and later the magnitude, angle and rotation desired. 
	 * It tells the motors what to do to achieve desired mag, angle and rotation.
	 * XBoxController accepts the index of the USB port, and can give the status of joysticks and buttons. 
	 */
	DriverStation ds = DriverStation.getInstance();

	double x, y, mag, theta, rotation;
	String gameData;
	MecanumDrive myRobot = new MecanumDrive(fLeft, bLeft, fRight, bRight);
	XboxController controller = new XboxController(0);

	/**
	 * Constants that are used so setpoint units can be easily converted to feet or wheel rotations (see below)
	 * Constants are found by using the ratio between setpoint values and pulses, and then pulses and wheel circumference
	 * Setpoint to degrees is found experimentally by feeding values and using the gyro to get the angle traveled.
	 */
	final double SETPOINT_TO_FEET = 36. / Math.PI;
	final double SETPOINT_TO_ROTATIONS = 18;
	final double SETPOINT_TO_DEGREES = 17. / 40;

	/**
	 * Declaring encoders, PID control modules and gyro. 
	 * Encoders take ports, inverted or not, and the encoding type, which is the ratio between cycles and pulses per revolution (1x, 2x, or 4x)
	 * The gyro gets the angle of the robot.
	 */
	double P, I, D, setpointCount;

	Encoder fLeftEnc = new Encoder(0, 1, true, CounterBase.EncodingType.k4X);
	Encoder bLeftEnc = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);
	Encoder fRightEnc = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
	Encoder bRightEnc = new Encoder(6, 7, false, CounterBase.EncodingType.k4X);

	DigitalInput lowerLim = new DigitalInput(8);
	DigitalInput upperLim = new DigitalInput(9);
	
	PIDController fLeftPID, bLeftPID, fRightPID, bRightPID, elevPID;
	int startPos = 0;
	boolean pants, scale1Left, scale2Left, scale3Left;
	Map<Double, Double> drivekP = new HashMap<>();
	Map<Double, Double> rotatekP = new HashMap<>();

	AHRS gyro = new AHRS(I2C.Port.kMXP);

	/**
	 * Robot Initialization- Runs when the robot is started
	 */
	@Override
	public void robotInit() {
		/**
		 * Encoders and gyro are zeroed when the robot starts.
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
		 * All relevant data is initialized on the SmartDashboard.
		 */
		SmartDashboard.putNumber("p", 0.012);
		SmartDashboard.putNumber("i", 0.00);
		SmartDashboard.putNumber("d", 500.00);

		SmartDashboard.putNumber("gyro", 0.00);
		SmartDashboard.putBoolean("gyroReset", false);

		SmartDashboard.putNumber("flEnc", 0);
		SmartDashboard.putNumber("blEnc", 0);
		SmartDashboard.putNumber("frEnc", 0);
		SmartDashboard.putNumber("brEnc", 0);
		SmartDashboard.putBoolean("encReset", false);

		SmartDashboard.putNumber("flDrive", 0);
		SmartDashboard.putNumber("frDrive", 0);
		SmartDashboard.putNumber("blDrive", 0);
		SmartDashboard.putNumber("brDrive", 0);
		SmartDashboard.putNumber("intake", 0);
		SmartDashboard.putNumber("intakerotate", 0);
		SmartDashboard.putNumber("elevator", 0);

		SmartDashboard.putNumber("timer", 135);
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
		SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + bLeft.getOutputCurrent() + fRight.getOutputCurrent() + bRight.getOutputCurrent());
		SmartDashboard.putNumber("intakedraw", intake1.getOutputCurrent() + intake2.getOutputCurrent());
		SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		SmartDashboard.putNumber("elevatordraw", elev1.getOutputCurrent() + elev2.getOutputCurrent());
		
		SmartDashboard.putNumber("velocity", Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY()));
		SmartDashboard.putNumber("acceleration", Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX()  + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY()));
		SmartDashboard.putNumber("temperature", gyro.getTempC());

		SmartDashboard.putBoolean("isred", ds.getAlliance().equals(Alliance.Red));
		SmartDashboard.putBoolean("scale1left", false);
		SmartDashboard.putBoolean("scale2left", false);
		SmartDashboard.putBoolean("scale3left", false);

		/**
		 * Automode refers to starting position. 0 = left, 1 = middle, 2 = right.
		 */
		SmartDashboard.putNumber("automode", 0);
		SmartDashboard.putBoolean("pants", false);

		/**
		 * Initialize PIDControllers with 0 for P, I & D, and encoders and talons for input and output.
		 */
		fLeftPID = new PIDController(0, 0, 0, fLeftEnc, fLeft);
		bLeftPID = new PIDController(0, 0, 0, bLeftEnc, bLeft);
		fRightPID = new PIDController(0, 0, 0, fRightEnc, fRight);
		bRightPID = new PIDController(0, 0, 0, bRightEnc, bRight);
		//elevPID = new PIDController(0, 0, 0, null, null);
				
		/**
		 * Map certain rotations and distances to kP values for autonomous.
		 */
		//.0412 - .0003166667 * d + .00000077778 * d * d
		drivekP.put(3.5, .012);
		drivekP.put(4.0, .012);
		drivekP.put(4.5, .012);
		drivekP.put(5.0, .012);
		drivekP.put(7.5, .012);
		drivekP.put(11.0, .012);
		drivekP.put(14.0, .012);
		drivekP.put(15.0, .012);
		drivekP.put(22.0, .012);
		drivekP.put(27.0, .012);
		
		rotatekP.put(90.0, .019);
		rotatekP.put(180.0, .0094);
		
		iRotate.setNeutralMode(NeutralMode.Brake);
	}

	/**
	 * Runs repeatedly while the robot is running.
	 * Mainly used to update data for the SmartDashboard.
	 */
	@Override
	public void robotPeriodic() {
		/**
		 * Checks if the reset buttons for gyro or encoders have been clicked, and resets if they have.
		 */
		if(SmartDashboard.getBoolean("gyroReset", false)) {
			gyro.reset();
			SmartDashboard.putBoolean("gyroReset", false);
		}
		SmartDashboard.putNumber("gyro", gyro.getAngle());

		if(SmartDashboard.getBoolean("encReset", false)) {
			fLeftEnc.reset();
			bLeftEnc.reset();
			fRightEnc.reset();
			bRightEnc.reset();
			SmartDashboard.putBoolean("encReset", false);
		}

		/**
		 * Updates encoder, motor, and other misc values for the SmartDashboard.
		 */
		SmartDashboard.putNumber("flEnc", fLeftEnc.getDistance() / 12);
		SmartDashboard.putNumber("blEnc", bLeftEnc.getDistance() / 12);
		SmartDashboard.putNumber("frEnc", fRightEnc.getDistance() / 12);
		SmartDashboard.putNumber("brEnc", bRightEnc.getDistance() / 12);

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
		SmartDashboard.putNumber("intake", intake1.get() * 100);
		SmartDashboard.putNumber("intakerotate", iRotate.get() * 100);
		SmartDashboard.putNumber("elevator", elev1.get() * 100);

		SmartDashboard.putNumber("timer", ds.getMatchTime());
		SmartDashboard.putBoolean("inauto", ds.isAutonomous());

		SmartDashboard.putNumber("voltage", RobotController.getBatteryVoltage());
		SmartDashboard.putNumber("totaldraw", PowerJNI.getVinCurrent());
		SmartDashboard.putNumber("drivedraw", fLeft.getOutputCurrent() + bLeft.getOutputCurrent() + fRight.getOutputCurrent() + bRight.getOutputCurrent());
		SmartDashboard.putNumber("intakedraw", intake1.getOutputCurrent() + intake2.getOutputCurrent());
		SmartDashboard.putNumber("intakerotatedraw", iRotate.getOutputCurrent());
		SmartDashboard.putNumber("elevatordraw", elev1.getOutputCurrent() + elev2.getOutputCurrent());
		
		SmartDashboard.putNumber("velocity", Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY()));
		SmartDashboard.putNumber("acceleration", Math.sqrt(gyro.getWorldLinearAccelX() * gyro.getWorldLinearAccelX()  + gyro.getWorldLinearAccelY() * gyro.getWorldLinearAccelY()));
		SmartDashboard.putNumber("temperature", gyro.getTempC());

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
		 */
		gameData = ds.getGameSpecificMessage();
		if(gameData.length() >= 3) {
			SmartDashboard.putBoolean("scale1left", gameData.charAt(0) == 'L');
			SmartDashboard.putBoolean("scale2left", gameData.charAt(1) == 'L');
			SmartDashboard.putBoolean("scale3left", gameData.charAt(2) == 'L');
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
		 */
		P = SmartDashboard.getNumber("p ", 0);
		I = SmartDashboard.getNumber("i", 0);
		D = SmartDashboard.getNumber("d", 0);
		setpointCount = 75;
		
		/**
		 * Puts the data on autonomous selection into variables, and then decides on and runs an auto.
		 */
		startPos = (int)SmartDashboard.getNumber("automode", 1);
		pants = SmartDashboard.getBoolean("pants", false);
		scale1Left = SmartDashboard.getBoolean("scale1left", false);
		scale2Left = SmartDashboard.getBoolean("scale2left", false);
		scale3Left = SmartDashboard.getBoolean("scale3left", false);

		/**
		 * Enables PIDContollers.
		 */
		fLeftPID.enable();
		bLeftPID.enable();
		fRightPID.enable();
		bRightPID.enable();
		
		/**
		 * Selects auto mode based on starting position, whether PANTS is enabled, and which side the scales are on.
		 */
		/*if(startPos == 0) {
			if(pants) {
				if(scale2Left) {
					autoDrive(27);
					autoRotate(90);
					autoDrive(3.5);
				}else {
					autoDrive(22);
					autoRotate(90);
					autoDrive(22);
					autoRotate(-90);
					autoDrive(5);
					autoRotate(-90);
					autoDrive(3.5);
				}
			}else {
				if(scale2Left) {
					autoDrive(27);
					autoRotate(90);
					autoDrive(3.5);
				}else if(scale1Left) {
					autoDrive(14);
					autoRotate(90);
					autoDrive(4);
				}else {
					autoDrive(14);
				}
			}
		}else if(startPos == 1) {
			if(pants) {
				if(scale2Left) {
					autoDrive(5);
					autoRotate(-90);
					autoDrive(11);
					autoRotate(90);
					autoDrive(22);
					autoRotate(90);
					autoDrive(3.5);
				}else {
					autoDrive(5);
					autoRotate(90);
					autoDrive(11);
					autoRotate(-90);
					autoDrive(22);
					autoRotate(-90);
					autoDrive(3.5);
				}
			}else {
				if(scale1Left) {
					autoDrive(5);
					autoRotate(-90);
					autoDrive(4.5);
					autoRotate(90);
					autoDrive(7.5);
				}else {
					autoDrive(5);
					autoRotate(90);
					autoDrive(4.5);
					autoRotate(-90);
					autoDrive(7.5);
				}
			}
		}else {
			if(pants) {
				if(scale2Left) {
					autoDrive(22);
					autoRotate(-90);
					autoDrive(22);
					autoRotate(90);
					autoDrive(5);
					autoRotate(90);
					autoDrive(3.5);
				}else {
					autoDrive(27);
					autoRotate(-90);
					autoDrive(3.5);
				}
			}else {
				if(!scale2Left) {
					autoDrive(27);
					autoRotate(-90);
					autoDrive(3.5);
				}else if(!scale1Left) {
					autoDrive(14);
					autoRotate(-90);
					autoDrive(4);
				}else {
					autoDrive(14);
				}
			}
		}*/
	}

	/**
	 * Runs repeatedly during autonomous. Can describe movement.
	 */
	@Override
	public void autonomousPeriodic() {
		
	}
	
	/**
	 * Teleop Initialization- Runs when driver controlled period starts
	 */
	@Override
	public void teleopInit() {
		/**
		 * PID is disabled so it will not interfere with driver control.
		 * Motors that had to be inverted for autonomous are un-inverted.
		 */
		fLeftPID.disable();
		bLeftPID.disable();
		fRightPID.disable();
		bRightPID. disable();

		fRight.setInverted(false);
		bRight.setInverted(false);
	}

	/**
	 * Teleop Periodic- Driver controls the robot
	 */
	@Override
	public void teleopPeriodic() {
		/**
		 * Takes controller input and calculates magnitude as movement. 
		 * Has dead zone because axes are not at exactly 0 when not touched.
		 * At the time of writing this code, the y axis is inexplicably inverted on XBox controllers, hence the (-).
		 */
		x = controller.getRawAxis(0);
		y = -controller.getRawAxis(1);
		if(Math.sqrt((x * x) + (y * y)) > 0.25)
			mag = Math.sqrt((x * x) + (y * y));
		else
			mag = 0;

		/**
		 * Rotation is taken from the right stick. It has a deadzone for the same reason.
		 */
		if(Math.abs(controller.getRawAxis(4)) > 0.25)
			rotation = controller.getRawAxis(4);
		else
			rotation = 0;

		/**
		 * The angle at which the robot should drive is determined here using basic trigonometry. 
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
		if(!controller.getBumper(Hand.kRight)) {
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
		 * Slows drive to 40% and rotation to 30% when left bumper is held down. Also for precise movement.
		 */
		if(controller.getBumper(Hand.kLeft))
			myRobot.drivePolar(mag * 0.3, theta, rotation * 0.2);
		else
			myRobot.drivePolar(mag, theta, rotation);

		/**
		 * When Y Button is held, the triggers control the elevator. Right for up, left for down.
		 * A small deadzone is added as usual.
		 */
		if(controller.getYButton()) {
			elev1.set(0);
			elev2.set(0);
			controller.setRumble(RumbleType.kLeftRumble, 0);
			controller.setRumble(RumbleType.kRightRumble, 0);
			if(controller.getTriggerAxis(Hand.kRight) > 0.2) {
				intake1.set(controller.getTriggerAxis(Hand.kRight));
				intake2.set(-controller.getTriggerAxis(Hand.kRight));
			}else if(controller.getTriggerAxis(Hand.kLeft) > 0.2) {
				intake1.set(-controller.getTriggerAxis(Hand.kLeft));
				intake2.set(controller.getTriggerAxis(Hand.kLeft));
			}else {
				intake1.set(0);
				intake2.set(0);
			}
		}else {
			/**
			 * Alternate control method for the intake. A and B move it at full power forwards and backwards.
			 */
			if(controller.getAButton()) {
				intake1.set(1.00);
				intake2.set(-1.00);
			}else if(controller.getBButton()) {
				intake1.set(-1.00);
				intake2.set(1.00);
			}else {
				intake1.set(0);
				intake2.set(0);
			}
			if(controller.getTriggerAxis(Hand.kRight) > 0.2) {
				elev1.set(-controller.getTriggerAxis(Hand.kRight));
				elev2.set(-controller.getTriggerAxis(Hand.kRight));
				controller.setRumble(RumbleType.kLeftRumble, controller.getTriggerAxis(Hand.kRight));
				controller.setRumble(RumbleType.kRightRumble, controller.getTriggerAxis(Hand.kRight));
			}else if(controller.getTriggerAxis(Hand.kLeft) > 0.2) {
				elev1.set(controller.getTriggerAxis(Hand.kLeft));
				elev2.set(controller.getTriggerAxis(Hand.kLeft));
				controller.setRumble(RumbleType.kLeftRumble, 0);
				controller.setRumble(RumbleType.kRightRumble, 0);
			}else {
				elev1.set(0);
				elev2.set(0);
				controller.setRumble(RumbleType.kLeftRumble, 0);
				controller.setRumble(RumbleType.kRightRumble, 0);
			}
		}
		
		/**
		 * Pivots the intake mechanism using the D Pad.
		 */
		if(controller.getPOV(0) != -1 && (controller.getPOV(0) >= 315 || controller.getPOV(0) <= 45))
			iRotate.set(.25); 
		else if(controller.getPOV(0) != -1 && controller.getPOV(0) >= 135 && controller.getPOV(0) <= 225)
			iRotate.set(-.25);
		else
			iRotate.set(0);
	}

	/**
	 * Test Initialization- Runs when test mode is started.
	 */
	@Override
	public void testInit() {
		
	}

	/**
	 * Used to test various robot functionality.
	 */
	@Override
	public void testPeriodic() {
		
	}

	/**
	 * Disabled Initialization- Runs when the robot is first disabled.
	 */
	@Override
	public void disabledInit() {

	}

	/**
	 * Runs repeatedly when the robot is disabled.
	 */
	@Override
	public void disabledPeriodic() {
		
	}

	public synchronized void autoDrive(double d) {
		new Thread(() -> {
			/**
			 * Encoder values are reset so the robot will go the distance of the setpoint from its current location.
			 * For auto drive, a constant P value of .012 is used. It was calculated experimentally with the robot.
			 */
			fLeftEnc.reset();
			bLeftEnc.reset();
			fRightEnc.reset();
			bRightEnc.reset();
			fLeftPID.setP(drivekP.get(Math.abs(d)));
			bLeftPID.setP(drivekP.get(Math.abs(d)));
			fRightPID.setP(drivekP.get(Math.abs(d)));
			bRightPID.setP(drivekP.get(Math.abs(d)));
			SmartDashboard.putNumber("p", drivekP.get(Math.abs(d)));

			/**
			 * For a smoother autonomous movement, the setpoint is split into smaller setpoints that 
			 * run in order every 25 milliseconds. Robot Periodic is called so values update during the loop.
			 * The loop is put in a separate thread so the field will not think the robot has crashed.
			 * Do this whenever you need to sleep code.
			 */
			for (int i = 1; i <= setpointCount; i++) {
				fLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
				bLeftPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
				fRightPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
				bRightPID.setSetpoint(((i * d) / setpointCount) * SETPOINT_TO_FEET);
				try{
					Thread.sleep(25);
				} catch(InterruptedException e) {}
			}
			fLeftPID.setSetpoint(d * SETPOINT_TO_FEET);
			bLeftPID.setSetpoint(d * SETPOINT_TO_FEET);
			fRightPID.setSetpoint(d * SETPOINT_TO_FEET);
			bRightPID.setSetpoint(d * SETPOINT_TO_FEET);
			try{
				Thread.sleep((int)D);
			} catch(InterruptedException e) {}
		}).start();
	}

	public synchronized void autoRotate(double d) {
		new Thread(() -> {
			/**
			 * Encoders are reset so it will rotate entirely from its current position.
			 * Unlike with auto drive, a P value is determined using an experimentally calculated quadratic regression.
			 * The purpose is so the robot will move with more precision when given low setpoints.
			 */
			fLeftEnc.reset();
			bLeftEnc.reset();
			fRightEnc.reset();
			bRightEnc.reset();
			fLeftPID.setP(rotatekP.get(Math.abs(d)));
			bLeftPID.setP(rotatekP.get(Math.abs(d)));
			fRightPID.setP(rotatekP.get(Math.abs(d)));
			bRightPID.setP(rotatekP.get(Math.abs(d)));
			SmartDashboard.putNumber("p", rotatekP.get(Math.abs(d)));

			/**
			 * The setpoint is split into smaller chunks for the same reason as above.
			 */
			for (int i = 1; i <= setpointCount; i++) {				
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
				Thread.sleep((int)D);
			} catch(InterruptedException e) {}
		}).start();
	}
}
