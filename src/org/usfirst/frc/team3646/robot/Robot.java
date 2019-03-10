package org.usfirst.frc.team3646.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {
	private DifferentialDrive myRobot;
	private VictorSP elevatorPos0, elevatorPos1, elevatorPos2, left, right, intakePos, intakeLeft, intakeRight;
	private SpeedControllerGroup elevatorPos;
	private Joystick joyDrive, joyTuscu;
	private Encoder elevatorEnc, intakeEnc, rightEnc, leftEnc;
	private ADXRS450_Gyro gyro;
	private DigitalInput floor0, floor1, ceiling, intakeLimit;
	private AnalogInput pressureSensor;
	private DoubleSolenoid elevatorShifter;
	private Solenoid elevatorLock;
	private int intakePosSet;
	private double mpwElevator, desiElevator, mpwIntake, desiIntake, parabolicInitialPos, parabolicTargetPos;
	private boolean isParabolicControl, check, intakeCalibration, isClimbMode, isShiftComplete, isLockMode, elevatorSupportDisable;
	private boolean isParabolicFinished, b1;
	private UsbCamera cam;
	private PowerDistributionPanel pdp;
	private Timer autoTimer,intakeTimer;
	@SuppressWarnings("rawtypes")
	SendableChooser autoChooser;

	@SuppressWarnings({ "unchecked", "rawtypes" })
	@Override
	public void robotInit() {
		cam = CameraServer.getInstance().startAutomaticCapture();
		cam.setFPS(15);
		cam.setResolution(320, 240);
		pdp = new PowerDistributionPanel();
		pdp.clearStickyFaults();
		left = new VictorSP(0);
		right = new VictorSP(1);
		myRobot = new DifferentialDrive(left, right);
		joyDrive = new Joystick(0);
		joyTuscu = new Joystick(1);
		elevatorPos0 = new VictorSP(2);
		elevatorPos1 = new VictorSP(3);
		elevatorPos2 = new VictorSP(8);
		elevatorPos = new SpeedControllerGroup(elevatorPos0, elevatorPos1, elevatorPos2);
		intakePos = new VictorSP(9);
		intakeRight = new VictorSP(6);
		intakeLeft = new VictorSP(7);
		elevatorShifter = new DoubleSolenoid(0, 1);
		elevatorLock = new Solenoid(4);
		rightEnc = new Encoder(10, 11, true, EncodingType.k4X);
		leftEnc = new Encoder(12, 13, false, EncodingType.k4X);
		elevatorEnc = new Encoder(16, 17, true, EncodingType.k4X);
		intakeEnc = new Encoder(18, 19, false, EncodingType.k4X);
		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		floor0 = new DigitalInput(0);
		floor1 = new DigitalInput(1);
		ceiling = new DigitalInput(2);
		intakeLimit = new DigitalInput(3);
		pressureSensor = new AnalogInput(0);
		intakePosSet = -1;
		isParabolicControl = false;
		check = false;
		intakeCalibration = true;
		isClimbMode = false;
		isShiftComplete = false;
		isLockMode = false;
		elevatorSupportDisable = false;
		isParabolicFinished = false;
		parabolicInitialPos = 0;
		parabolicTargetPos = 0;
		elevatorEnc.setDistancePerPulse(0.0413);
		intakeEnc.setDistancePerPulse(0.4);
		rightEnc.setDistancePerPulse(4 * 2.54 * Math.PI / 900);
		leftEnc.setDistancePerPulse(4 * 2.54 * Math.PI / 900);
		autoChooser = new SendableChooser();
		autoChooser.addDefault("Switch", 1);
		autoChooser.addObject("Auto Line Pass", 2);
		autoChooser.addObject("Scale Main", 3);
		autoChooser.addObject("Scale Only Right", 4);
		autoChooser.addObject("Scale and Switch", 5);
		autoChooser.addObject("Fill Air Tank", 6);
		autoChooser.addObject("Practice Left Switch", 7);
		autoChooser.addObject("Practice Right Scale", 8);
		autoChooser.addObject("Practice Left Scale", 9);
		autoChooser.addObject("Swamp or Die Edition", 10);
		SmartDashboard.putData("Autonomous Chooser", autoChooser);
		intakeTimer = new Timer();
	}

	@Override
	public void autonomousInit() {
		int mode = 1;
		mode = (int) autoChooser.getSelected();
		// For air filling
		if(mode != 6)
			elevatorLock.set(true);
		intakeTimer.reset();
		intakeTimer.start();
		
		encReset();
		gyro.reset();
		
		switch (mode) {

		//Switch
		case 1:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
				intakeCalibration();
				switchRightAutoEnc();
			}
			else {
				intakeCalibration();
				switchLeftAutoEnc();
			}
			break;
			
		// Auto line pass
		case 2:
			intakeCalibration();
			while (leftEnc.getDistance() <= 350)
				myRobot.arcadeDrive(-0.65, 0);
			myRobot.stopMotor();
			break;
			
		// Scale left and right - main
		case 3:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'R')
				scaleRightAutoGyro();
			else {
				scaleLeftAutoGyro();
				scaleLeftPart2();
			}
			break;
		
		//Scale only right
		case 4:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'R')
				scaleRightAutoGyro();
			else if(DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R'){
				intakeCalibration();
				switchRightAutoGyro();
			} else {
				intakeCalibration();
				while (leftEnc.getDistance() <= 350)
					myRobot.arcadeDrive(-0.65, 0);
				myRobot.stopMotor();
			}
			break;
		
		//Scale then switch or left
		case 5:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'R' && 
					DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R')
				uselessAuto();
			else if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'R')
				scaleRightAutoGyro();
			else {
				scaleLeftAutoGyro();
				scaleLeftPart2();
			}
			break;
		case 6:
			//Fill air
			break;

		//Left Switch Practice
		case 7:
			intakeCalibration();
			switchLeftAutoEnc();
			break;

		//Right Scale Practice
		case 8:
			scaleRightAutoGyro();
			break;

		//Left Scale Practice
		case 9:
			scaleLeftAutoGyro();
			scaleLeftPart2();
			break;

		//Swamp
		case 10:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(1) == 'L')
				leftScaleSwamp();
			else if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'L') {
				intakeCalibration();
				leftSwitchSwamp();
			}
			else {
				intakeCalibration();
				while (leftEnc.getDistance() <= 330)
					myRobot.arcadeDrive(-0.65, 0);
				myRobot.stopMotor();
			}
			break;

		default:
			if (DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R') {
				intakeCalibration();
				switchRightAutoEnc();
			}
			else
			{
				intakeCalibration();
				switchLeftAutoEnc();
			}
			break;
		}
	}

	@Override
	public void autonomousPeriodic() {
		putAllData();
		myRobot.stopMotor();
	}

	@Override
	public void teleopInit() {
		elevatorLock.set(true);
		gyro.reset();
		//elevatorEnc.reset();
		//intakeEnc.reset();
		mpwElevator = 0;
		mpwIntake = 0;
		parabolicInitialPos = 0;
		parabolicTargetPos = 0;
		intakePosSet = -1;
		desiElevator = elevatorEnc.getDistance();
		desiIntake = intakeEnc.getDistance();
		isParabolicControl = false;
		check = false;
		isClimbMode = false;
		b1 = false;
		intakeTimer.reset();
		intakeTimer.start();
	}

	@Override
	public void teleopPeriodic() {
		putAllData();

		// Elevator speed limiting
		if (elevatorEnc.getDistance() >= 120)
			myRobot.arcadeDrive((joyDrive.getRawAxis(1) * (elevatorEnc.getDistance() * -0.00417 + 1.5)) , (joyDrive.getRawAxis(4) * (elevatorEnc.getDistance() * -0.00292 + 1.3504)));
		else
			myRobot.arcadeDrive(joyDrive.getRawAxis(1), joyDrive.getRawAxis(4));

		// Intake Button controls
		if (joyDrive.getRawAxis(2) > 0.9 || joyTuscu.getRawAxis(2) > 0.9) { //Disari atma
			intakeLeft.set(0.5);
			intakeRight.set(-0.5);
			check = false;
		}
		else if (joyDrive.getRawAxis(2) > 0.1 || joyTuscu.getRawAxis(2) > 0.1) { //Disari atma
			intakeLeft.set(0.27);
			intakeRight.set(-0.27);
			check = false;
		}
		else if (joyDrive.getRawButton(5)) { // Yavasca atma
			intakeLeft.set(0.25);
			intakeRight.set(-0.25);
			check = false;
		}
		else if (joyTuscu.getRawButton(5)) { //Icine alma
			intakeLeft.set(-0.5);
			intakeRight.set(0.5);
		}
		else if (joyDrive.getRawButton(6) || joyTuscu.getRawAxis(3) > 0.2) { //Oldugu yere birakma
			intakeLeft.set(0.43);
			intakeRight.set(0.43);
			check = false;
		}
		else if (joyTuscu.getRawButtonReleased(5)) { // Stall
			intakeLeft.set(-0.1);
			intakeRight.set(0.1);
			check = true;
		}
		else if (!check && (joyDrive.getRawButtonReleased(5) || 
				joyDrive.getRawAxis(2) < 0.1 || joyTuscu.getRawAxis(2) < 0.1)) {
			intakeLeft.set(0);
			intakeRight.set(0);
		}
		
		// Elevator position set and stabilizing
		mpwElevator = (desiElevator - elevatorEnc.getDistance()) * 0.055;
		desiElevator += (Math.abs(joyTuscu.getRawAxis(5)) > 0.1) && (desiElevator >= 0 && desiElevator <= 245) ?  -joyTuscu.getRawAxis(5) * 1.62 : 0;
		if (desiElevator <= 0)
			desiElevator = 0;
		if (desiElevator > 245)
			desiElevator = 245;
		if (!ceiling.get())
			desiElevator = 245;
		
		// Elevator and intake automatic positions
		if (joyTuscu.getRawButtonPressed(1)) {
			parabolicInitialPos = elevatorEnc.getDistance();
			parabolicTargetPos = 0;
			intakePosSet = 93;
			isParabolicControl = true;
		}
		
		if (joyTuscu.getRawButtonPressed(2)) {
			parabolicInitialPos = elevatorEnc.getDistance();
			parabolicTargetPos = 60;
			intakePosSet = 45;
			isParabolicControl = true;
		}
		
		if (joyTuscu.getRawButtonPressed(3)) {
			parabolicInitialPos = elevatorEnc.getDistance();
			parabolicTargetPos = 156;
			intakePosSet = 55;
			isParabolicControl = true;
		}

		if (joyTuscu.getRawButtonPressed(4)) {
			parabolicInitialPos = elevatorEnc.getDistance();
			parabolicTargetPos = 185;
			intakePosSet = 55;
			isParabolicControl = true;
		}
		
		if (joyTuscu.getRawButtonPressed(6)) {
			parabolicInitialPos = elevatorEnc.getDistance();
			parabolicTargetPos = 230;
			intakePosSet = 45;
			isParabolicControl = true;
		}
		
		//Climb Mode Initiate
		if (joyTuscu.getRawButtonPressed(8)) {
			isClimbMode = true;
			parabolicInitialPos = elevatorEnc.getDistance();
			elevatorShifter.set(DoubleSolenoid.Value.kReverse);
			parabolicTargetPos = 215;
			isParabolicControl = true;
		}
		
		//Disable climb mode
		if(joyTuscu.getRawButton(7)) {
			elevatorLock.set(true);
			elevatorShifter.set(DoubleSolenoid.Value.kForward);
			isClimbMode = false;
			isShiftComplete = false;
			elevatorSupportDisable = false;
			b1 = false;
			desiElevator = elevatorEnc.getDistance();
		}
		
		// Climb mode
		if(isClimbMode && !isShiftComplete){
			elevatorShifter.set(DoubleSolenoid.Value.kReverse);
			isShiftComplete = true;
			isClimbMode = false;
			elevatorSupportDisable = true;
		}
		
		if(joyDrive.getRawButtonPressed(7)) {
			elevatorShifter.set(DoubleSolenoid.Value.kForward);
		}
		
		
		else if (isShiftComplete && elevatorEnc.getDistance() >= 50 && intakeEnc.getDistance() > 70) {
			intakePosSet = 180;
		}
		else if (isShiftComplete && intakeEnc.getDistance() < 70) {
			intakePosSet = 180;
		} 
		else if(isShiftComplete && elevatorEnc.getDistance() >= 214) {
			elevatorPos.set(0);
			isParabolicControl = false;
		}
		
		//Lock elevator
		if(isLockMode && elevatorEnc.getDistance() > 170) {
			elevatorLock.set(false); //lock it
			isLockMode = false;
			isShiftComplete = false;
		}
		
		//firstDriverClimb
		if(!isAtFloor() && elevatorEnc.getDistance() > 70 && joyDrive.getRawButton(4)) { 
			elevatorPos.set(-0.0125 * elevatorEnc.getDistance() + 3.25);
			isLockMode = true;
		} else if(joyDrive.getRawButtonReleased(4) || elevatorEnc.getDistance() < 70) { 
			elevatorPos.set(0);
			isLockMode = false;
		}

		// Intake automatic modes
		if (joyTuscu.getRawButton(9)) intakePosSet = 0;
		else if (joyTuscu.getPOV() == 0) intakePosSet = 0;
		else if (joyTuscu.getPOV() == 90) intakePosSet = 90;
		else if (joyTuscu.getPOV() == 180) intakePosSet = 45;
		else if (joyTuscu.getPOV() == 270) intakePosSet = 80;
		
		// Elevator
		if (isParabolicControl) {
			//below 45 up
			if (parabolicTargetPos - parabolicInitialPos > 0 && parabolicTargetPos - parabolicInitialPos < 45) {
				if (elevatorEnc.getDistance() - parabolicInitialPos <= parabolicTargetPos - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.4, elevatorEnc.getDistance()) - 0.17);	
				}
				else
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.35, elevatorEnc.getDistance()) - 0.15);
			}
			//below 65 up
			else if (parabolicTargetPos - parabolicInitialPos > 0 && parabolicTargetPos - parabolicInitialPos < 65) {
				if (elevatorEnc.getDistance() - parabolicInitialPos <= parabolicTargetPos - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.55, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.5, elevatorEnc.getDistance()) - 0.17);
			}
			//145 to 225
			else if (parabolicTargetPos == 225 && parabolicInitialPos > 145) {
				if (elevatorEnc.getDistance() - parabolicInitialPos <= parabolicTargetPos - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.5, elevatorEnc.getDistance()) - 0.17);	
				}
				else
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.45, elevatorEnc.getDistance()) - 0.15);
			}		
			//Switch to up
			else if ((parabolicTargetPos - parabolicInitialPos) > 0 && parabolicTargetPos > 150 && parabolicInitialPos > 50) {
				if (elevatorEnc.getDistance() - parabolicInitialPos <= parabolicTargetPos - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.8, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.75, elevatorEnc.getDistance()) - 0.15);
			}
			//below 45 down
			else if(parabolicTargetPos - parabolicInitialPos < 0 && parabolicInitialPos - parabolicTargetPos  < 45) {
				elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.2, elevatorEnc.getDistance()) + 0.07);
			}
			
			//below 75 down
			else if(parabolicTargetPos - parabolicInitialPos < 0 && parabolicInitialPos - parabolicTargetPos  < 75) {
				elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.4, elevatorEnc.getDistance()) + 0.1);
			}
			
			//to switch down
			else if(parabolicTargetPos == 60 && parabolicTargetPos - parabolicInitialPos < 0 && parabolicInitialPos > 150) {
				elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.45, elevatorEnc.getDistance()) + 0.1);
			}
			
			//up
			else if (parabolicInitialPos - parabolicTargetPos < 0) {
				if (elevatorEnc.getDistance() - parabolicInitialPos <= parabolicTargetPos - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 1.5, elevatorEnc.getDistance()) - 0.32);	
				}
				else
					elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			//down
			else 
				elevatorPos.set(parabolicControl(parabolicInitialPos, parabolicTargetPos, 0.7, elevatorEnc.getDistance()) + 0.12);


			//Stop in parabolic
			if (Math.abs(elevatorEnc.getDistance() - parabolicTargetPos) < 2 || joyTuscu.getRawButtonPressed(10) || 
					elevatorEnc.getDistance() > 240 || !ceiling.get() 
			    		|| (parabolicTargetPos == 0 && isAtFloor())) {
				if (parabolicTargetPos == 0)
					desiElevator = 0;
				else {
					desiElevator = elevatorEnc.getDistance();
					mpwElevator = 0.07;
				}
				parabolicInitialPos = 0;
				parabolicTargetPos = 0;
				isParabolicControl = false;
			}
		}

		// Elevator manual control
		if (!isParabolicControl) {
			if(!elevatorSupportDisable) {
				if (isAtFloor() && desiElevator < 0.2) {
					elevatorPos.set(0);
				}
				else if (ceiling.get()){
					elevatorPos.set(Math.abs(mpwElevator) < 0.7 ? -mpwElevator : -(Math.abs(mpwElevator) / mpwElevator) * 0.7);
				}
			} else {
				if (isAtFloor() && desiElevator < 0.2) {
					elevatorPos.set(0);
				}
				else if (ceiling.get()){
					if(mpwElevator < 0 && !joyDrive.getRawButton(4))
					elevatorPos.set(-mpwElevator * 0.7);
				} else if(!ceiling.get()) {
					elevatorPos.set(0);
				}
			}
		}
		
		//Intake Control
		if(intakeCalibration) {
			intakePos.set(-0.25);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakeCalibration = false;
			}
			if(intakeTimer.get() > 1.25) {
				intakeCalibration = false;
			}
		} else {
			desiIntake = desiIntake < 0 ? 0 : desiIntake;
			if(intakeEnc.getDistance() > 160 && elevatorSupportDisable) {
				b1 = true;
			} else if(!b1) {
				mpwIntake = (desiIntake - intakeEnc.getDistance()) * 0.04; // Intake sabit tutma
			}
			
			if(intakeEnc.getDistance() > 140 && elevatorSupportDisable) {
				mpwIntake = 0;
				intakePos.set(0);
			}

			if (desiIntake >= 0 && desiIntake <= 180 && !b1) {
				desiIntake += Math.abs(joyTuscu.getRawAxis(1)) > 0.1 ?  joyTuscu.getRawAxis(1) / 0.5 : 0;
			}
			if (!intakeCalibration && intakePosSet == -1 && !b1) {
				intakePos.set(Math.abs(mpwIntake) > 0.035 ? mpwIntake : 0);
			}
			else if (Math.abs(intakeEnc.getDistance() - intakePosSet) > 3 && !b1) {
				intakePos.set(intakeEnc.getDistance() - intakePosSet < 0 ? 0.18 : -0.375);
			}
			else if (Math.abs(intakeEnc.getDistance() - intakePosSet) <= 4  && !b1) {
				intakePosSet = -1;
				desiIntake = intakeEnc.getDistance();
			}
		}

	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void disabledPeriodic() {
		putAllData();
	}

	private boolean isAtFloor() {
		return !floor0.get() || !floor1.get();
	}

	private double parabolicControl(double initialPos, double targetPos, double maxSpeed, double currentPos) { // Motor power acceleration control
		double yVal = (currentPos - initialPos) * (currentPos - targetPos);
		double b = -(initialPos + targetPos);
		double c = initialPos * targetPos;
		double vertexPoint = (4 * c - b * b) / 4;
		return targetPos - initialPos < 0 ? maxSpeed * yVal / vertexPoint : -maxSpeed * yVal / vertexPoint;
	}

	private void putAllData() {
		SmartDashboard.putNumber("Elevator Distance", elevatorEnc.getDistance());
		SmartDashboard.putNumber("Intake Distance", intakeEnc.getDistance());
		SmartDashboard.putNumber("Right Drive Distance", rightEnc.getDistance());
		SmartDashboard.putNumber("Left Drive Distance", leftEnc.getDistance());
		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
		
		
		
		SmartDashboard.putBoolean("Elevator Up Switch", !ceiling.get());
		SmartDashboard.putBoolean("Floor Switch 0", !floor0.get());
		SmartDashboard.putBoolean("Floor Switch 1", !floor1.get());
		SmartDashboard.putBoolean("Intake Switch", !intakeLimit.get());
		
		SmartDashboard.putNumber("Pressure Sensor", pressureSensor.getVoltage());

		SmartDashboard.putNumber("mpwElevator", mpwElevator);
		SmartDashboard.putNumber("desiElevator", desiElevator);
		SmartDashboard.putNumber("mpwIntake", mpwIntake);
		SmartDashboard.putNumber("desiIntake", desiIntake);
		
		SmartDashboard.putNumber("Current", pdp.getTotalCurrent());
		
		SmartDashboard.putNumber("v", 7.1);
	}

	private void switchRightAutoEnc() {
		autoTimer = new Timer();
		autoTimer.reset();
		intakeLeft.set(-0.12);
		intakeRight.set(0.12);
		while (leftEnc.getDistance() < 160 && isAutonomous()) { 
			
			//Lift
			if(elevatorEnc.getDistance() < 65 && !isParabolicFinished) {
				if (elevatorEnc.getDistance() - 0 <= 65 - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(0, 65, 0.45, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(0, 65, 0.4, elevatorEnc.getDistance()) - 0.17);
			} 
			//Stay stable
			else {
				elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
				isParabolicFinished = true;
			}
			
			//Intake stuff
			if (Math.abs(intakeEnc.getDistance() - 45) > 3) {
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.1658 : -0.3);
			}
			// Intake sabit tutma
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 ) {
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			} 
			
			myRobot.arcadeDrive(-0.85, 0);
			Timer.delay(0.02);
		}
		
		autoTimer.start();
		while (autoTimer.get() < 1.5 && isAutonomous()) {
			myRobot.arcadeDrive(-0.4, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}

		autoTimer.reset();
		autoTimer.start();
		//Shoot the box
		while(autoTimer.get() < 0.5 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.3);
			intakeRight.set(-0.3);
			elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		
		intakeLeft.set(0);
		intakeRight.set(0);
		
		//Drive back
		encReset();
		while(leftEnc.getDistance() > -30 && isAutonomous()) {
			myRobot.arcadeDrive(0.5, 0);
			if(intakeEnc.getDistance() > 3) {
				intakePos.set(-0.25);
			}
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Close intake
		while(intakeEnc.getDistance() > 3 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.25);
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		
		//Elevator Down
		while(!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			Timer.delay(0.02);
		}
		
		elevatorPos.set(0);
		intakePos.set(0);
		myRobot.stopMotor();
	}

	private void switchLeftAutoEnc() {
		autoTimer = new Timer();
		intakeLeft.set(-0.12);
		intakeRight.set(0.12);
		
		while (encAvg() < 20 && isAutonomous()) {
			myRobot.arcadeDrive(-0.65, 0);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		autoTimer.reset();
		autoTimer.start();
		while(autoTimer.get() < 0.25 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.15);
		}
		
		encTurn(68, 0.8, true);
		
		autoTimer.reset();
		autoTimer.start();
		while(autoTimer.get() < 0.25 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.15);
		}
		
		intakePos.set(0);
		
		while (encAvg() < 205 && isAutonomous()) {
			myRobot.arcadeDrive(-0.8, 0);
			Timer.delay(0.02);
		}
		
		autoTimer.reset();
		autoTimer.start();
		while(autoTimer.get() < 0.25 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.15);
		}
	
		encTurn(82, 0.8, false);
		
		autoTimer.reset();
		autoTimer.start();
		while(autoTimer.get() < 0.25 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.15);
		}
		
		autoTimer.reset();
		autoTimer.start();	
		//Go forward get in switch position
		while (autoTimer.get() < 1.5 && isAutonomous()) {
			myRobot.arcadeDrive(-0.58, 0);
			
			if (Math.abs(intakeEnc.getDistance() - 45) > 3) {
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.18 : -0.375);
			}
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 ) {
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			} 
			
			if (Math.abs(elevatorEnc.getDistance() - 65) > 2 ) {
				if (elevatorEnc.getDistance() - 0 <= 65 - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(0, 65, 0.45, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(0, 65, 0.4, elevatorEnc.getDistance()) - 0.17);
			}
			else {
				elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.055);
			}
			Timer.delay(0.02);
		}

		autoTimer.reset();
		autoTimer.start();
		//Shoot the box
		while(autoTimer.get() < 0.5 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.32);
			intakeRight.set(-0.32);
			elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.055);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}

		intakeLeft.set(0);
		intakeRight.set(0);

		//Drive back
		encReset();
		while(leftEnc.getDistance() > -33 && isAutonomous()) {
			myRobot.arcadeDrive(0.5, 0);
			if (intakeEnc.getDistance() > 3) {
				intakePos.set(-0.25);
			}
			if (!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.3, elevatorEnc.getDistance()) + 0.08);
			}
			Timer.delay(0.02);
		}

		//Close intake
		while(intakeEnc.getDistance() > 3 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.25);
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.3, elevatorEnc.getDistance()) + 0.08);
			}
			Timer.delay(0.02);
		}


		//Elevator Down
		while(!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			Timer.delay(0.02);
		}

		elevatorPos.set(0);
		intakePos.set(0);
		myRobot.stopMotor();
	}

	private void scaleRightAutoGyro() {
		Timer t = new Timer();
		t.reset();
		
		// Go Forward
		encReset();
		while (encAvg() < 270 && isAutonomous()) {
			myRobot.arcadeDrive(-1, 0);
			intakePos.set(-0.2);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakePos.set(0);
				intakeCalibration = false;
			}
			if(intakeTimer.get() > 0.5) {
				intakePos.set(0);
				intakeCalibration = false;
			}
			Timer.delay(0.02);
		}
		while (encAvg() < 570 && isAutonomous()) {
			myRobot.arcadeDrive(encAvg() * 0.001 - 1.27, 0);
			Timer.delay(0.02);
		}
		
		stopRobot(0.2, true);

		//Gyro turn for first box, lift elevator
		while (gyro.getAngle() > -48 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.75);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);

		}
		myRobot.stopMotor();
		while (gyro.getAngle() < -48 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.7);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		
		}
		
		// Set Positions
		while (elevatorEnc.getDistance() < 213 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(-0.1);
			intakeRight.set(0.1);
			// Elevator
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			
			Timer.delay(0.02);
		}

		// Go Forward
		encReset();
		while (encAvg() < 33 && isAutonomous()) {
			myRobot.arcadeDrive(-0.6, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Shoot
		t.reset();
		t.start();
		while(t.get() < 0.3 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.24);
			intakeRight.set(-0.24);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		// Go Back
		encReset();
		while (encAvg() > -27 && isAutonomous()) {
			myRobot.arcadeDrive(0.7, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Set Positions
		double tempPos = elevatorEnc.getDistance();
		while (!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(tempPos, 0, 0.7, elevatorEnc.getDistance()) + 0.12);
			else
				elevatorPos.set(0);
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 95) < 4)
				intakePos.set(0.18);
			else
				intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		
		// Go Forward
		encReset();
		while (encAvg() < 5 && isAutonomous()) {
			myRobot.arcadeDrive(-0.75, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		stopRobot(0.1, true);

		// Turn to Box
		gyro.reset();
		while (gyro.getAngle() > -137 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.85);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
		}
		
		while (gyro.getAngle() < -137 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.8);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
		}
		myRobot.stopMotor();
		stopRobot(0.1, true);
		
		// Go Forward and Take the Power Cube
		encReset();
		while (encAvg() < 70 && isAutonomous()) {
			myRobot.arcadeDrive(-0.8, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		t.reset();
		t.start();
		while (t.get() < 0.8 && isAutonomous()) {
			myRobot.arcadeDrive(-0.5, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			intakeLeft.set(-0.6);
			intakeRight.set(0.6);
		}
		myRobot.stopMotor();
		
		gyro.reset();
		encReset();
		
		//Go back a little
		while (encAvg() > -20 && isAutonomous()) {
			myRobot.arcadeDrive(0.6, 0.5);
			intakeLeft.set(-0.42);
			intakeRight.set(0.42);
			if (Math.abs(intakeEnc.getDistance() - 85) > 3)
				intakePos.set(intakeEnc.getDistance() - 85 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 85) <= 4 )
				intakePos.set((85 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		// Go Back with cube
		encReset();
		while (encAvg() > -50 && isAutonomous()) {
			myRobot.arcadeDrive(0.8, 0.5);
			intakeLeft.set(-0.33);
			intakeRight.set(0.33);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.3, elevatorEnc.getDistance()) - 0.27);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.4, elevatorEnc.getDistance()) - 0.12);
			}
			if (Math.abs(intakeEnc.getDistance() - 55) > 3)
				intakePos.set(intakeEnc.getDistance() - 55 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 55) <= 4 )
				intakePos.set((55 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		encReset();
		while (encAvg() > -50 && isAutonomous()) {
			myRobot.arcadeDrive(0.75, 0);
			intakeLeft.set(-0.25);
			intakeRight.set(0.25);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.3, elevatorEnc.getDistance()) - 0.27);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.4, elevatorEnc.getDistance()) - 0.12);
			}
			if (Math.abs(intakeEnc.getDistance() - 55) > 3)
				intakePos.set(intakeEnc.getDistance() - 55 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 55) <= 4 )
				intakePos.set((55 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		//Stall intake
		intakeLeft.set(-0.125);
		intakeRight.set(0.125);
		
		// Turn to Scale
		
		while (gyro.getAngle() < 82
				&& isAutonomous()) {
			myRobot.arcadeDrive(0, 0.8);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.3, elevatorEnc.getDistance()) - 0.27);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.4, elevatorEnc.getDistance()) - 0.12);
			}
			if (Math.abs(intakeEnc.getDistance() - 55) > 3)
				intakePos.set(intakeEnc.getDistance() - 55 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 55) <= 4 )
				intakePos.set((55 - intakeEnc.getDistance()) * 0.04);
		}
		
		myRobot.stopMotor();

		// Go Forward
		encReset();
		while (encAvg() < 30 && isAutonomous()) {
			myRobot.arcadeDrive(-0.7, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.3, elevatorEnc.getDistance()) - 0.27);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.4, elevatorEnc.getDistance()) - 0.12);
			}
			intakePos.set((55 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();

		// Shoot second cube
		t.reset();
		t.start();
		while(t.get() < 0.75 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.3);
			intakeRight.set(-0.3);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((55 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);

		// Go Back
		encReset();
		while (encAvg() > -30 && isAutonomous()) {
			myRobot.arcadeDrive(0.75, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((55 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		

		// Set Positions
		tempPos = elevatorEnc.getDistance();
		while (!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(tempPos, 0, 0.7, elevatorEnc.getDistance()) + 0.07);
			else
				elevatorPos.set(0);

			if (Math.abs(intakeEnc.getDistance() - 20) > 3)
				intakePos.set(intakeEnc.getDistance() - 20 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 20) <= 4 )
				intakePos.set((20 - intakeEnc.getDistance()) * 0.04);
			
			Timer.delay(0.02);
		}
		
		t.reset();
		t.start();
		while(t.get() < 0.5 && isAutonomous()) {
			intakePos.set(-0.15);
			myRobot.stopMotor();
		}
		
		intakePos.set(0);
	}

	private void switchRightAutoGyro() {
		Timer autoTimer = new Timer();
		autoTimer.reset();
		
		// Go Forward
		encReset();
		while (encAvg() < 325 && isAutonomous()) {
			myRobot.arcadeDrive(0.001 * encAvg() - 0.95 , 0);
			Timer.delay(0.02);
		}

		//Gyro turn for switch
		while (gyro.getAngle() > -90 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.75);
			
			//Elevator
			if (Math.abs(elevatorEnc.getDistance() - 60) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 60 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 60, 0.55, elevatorEnc.getDistance()) - 0.2);
				else
					elevatorPos.set(parabolicControl(0, 60, 0.5, elevatorEnc.getDistance()) - 0.17);
			}

			// Intake
			if (Math.abs(intakeEnc.getDistance() - 45) > 3)
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 )
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04);

		}
		
		myRobot.stopMotor();
		while (gyro.getAngle() < -90 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.65);
			//Elevator
			if (Math.abs(elevatorEnc.getDistance() - 60) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 60 - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(0, 60, 0.55, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(0, 60, 0.5, elevatorEnc.getDistance()) - 0.17);
			}

			// Intake
			if (Math.abs(intakeEnc.getDistance() - 45) > 3)
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 )
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
		}
		
		//Forward
		autoTimer.reset();
		autoTimer.start();
		while (autoTimer.get() < 1.25 && isAutonomous()) {
			myRobot.arcadeDrive(-0.4, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}

		autoTimer.reset();
		autoTimer.start();
		
		//Shoot the box
		while(autoTimer.get() < 0.7 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.25);
			intakeRight.set(-0.25);
			elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		//Drive back
		encReset();
		while(leftEnc.getDistance() > -33 && isAutonomous()) {
			myRobot.arcadeDrive(0.45, 0);
			if(intakeEnc.getDistance() > 5) {
				intakePos.set(-0.25);
			}
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Close intake
		while(intakeEnc.getDistance() > 3 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.2);
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Elevator Down
		while(!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			Timer.delay(0.02);
		}
		
		elevatorPos.set(0);
		intakePos.set(0);
		myRobot.stopMotor();
	}

	private void scaleLeftAutoGyro() {
		Timer autoTimer = new Timer();
		autoTimer.reset();
		
		// Go Forward
		encReset();
		while (encAvg() < 250 && isAutonomous()) {
			myRobot.arcadeDrive(-0.9, 0);
			intakePos.set(-0.2);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakePos.set(0);
				intakeCalibration = false;
			}
			if (intakeTimer.get() > 0.5) {
				intakePos.set(0);
				intakeCalibration = false;
			}
			Timer.delay(0.02);
		}
		
		while (encAvg() < 520 && isAutonomous()) {
			myRobot.arcadeDrive(encAvg() * 0.001 - 1.2, 0);
			Timer.delay(0.02);
		}

		stopRobot(0.35, true);

		//Gyro turn for platform zone
		while (gyro.getAngle() > -99 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.75);
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 0) > 3)
				intakePos.set(intakeEnc.getDistance() - 0 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 0) <= 4 )
				intakePos.set((0 - intakeEnc.getDistance()) * 0.04);
		}
		
		myRobot.stopMotor();
		while (gyro.getAngle() < -99 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.65);
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 0) > 3)
				intakePos.set(intakeEnc.getDistance() - 0 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 0) <= 4 )
				intakePos.set((0 - intakeEnc.getDistance()) * 0.04);
		}
		
		stopRobot(0.35, true);
		
		// go to part 2
	}
	
	private void scaleLeftPart2() {
		encReset();
		while (encAvg() < 350 && isAutonomous())
			myRobot.arcadeDrive(-0.75, 0);
		while (encAvg() < 550 && isAutonomous()) 
			myRobot.arcadeDrive(-1.05 + 0.001 * encAvg(), 0);
		stopRobot(0.3, true);

		//Gyro turn for scale plate
		gyro.reset();
		while (gyro.getAngle() < 116 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.75);
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		}
		myRobot.stopMotor();
		while (gyro.getAngle() > 116 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.65);
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		}
		myRobot.stopMotor();

		intakeLeft.set(-0.11);
		intakeRight.set(0.11);
		while (Math.abs(elevatorEnc.getDistance() - 215) < 3 && isAutonomous()) {
			myRobot.stopMotor();
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		}

		// Go Forward
		encReset();
		while (encAvg() < 85 && isAutonomous()) {
			myRobot.arcadeDrive(-0.5, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		
		// Shoot
		Timer t = new Timer();
		t.reset();
		t.start();
		
		while(t.get() < 0.25 && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		}
		
		
		while(t.get() < 1 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.38);
			intakeRight.set(-0.38);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		// Go Back
		encReset();
		while (encAvg() > -65 && isAutonomous()) {
			myRobot.arcadeDrive(0.5, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		

		// Set Positions
		while (!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(215, 0, 0.7, elevatorEnc.getDistance()) + 0.07);
			else
				elevatorPos.set(0);

			if (Math.abs(intakeEnc.getDistance() - 20) > 3)
				intakePos.set(intakeEnc.getDistance() - 20 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 20) <= 4 )
				intakePos.set((20 - intakeEnc.getDistance()) * 0.04);
			
			Timer.delay(0.02);
		}
		
		t.reset();
		t.start();
		while(t.get() < 0.5 && isAutonomous()) {
			intakePos.set(-0.15);
			myRobot.stopMotor();
		}
		
		intakePos.set(0);
		
	}

	private void uselessAuto() {
		Timer t = new Timer();
		t.reset();
		
		// Go Forward
		encReset();
		while (encAvg() < 270 && isAutonomous()) {
			myRobot.arcadeDrive(-1, 0);
			intakePos.set(-0.2);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakePos.set(0);
				intakeCalibration = false;
			}
			if(intakeTimer.get() > 0.5) {
				intakePos.set(0);
				intakeCalibration = false;
			}
			Timer.delay(0.02);
		}
		while (encAvg() < 570 && isAutonomous()) {
			myRobot.arcadeDrive(encAvg() * 0.001 - 1.27, 0);
			Timer.delay(0.02);
		}
		
		stopRobot(0.2, true);

		//Gyro turn for first box, lift elevator
		while (gyro.getAngle() > -48 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.75);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);

		}
		myRobot.stopMotor();
		while (gyro.getAngle() < -48 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.7);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
		
		}
		
		// Set Positions
		while (elevatorEnc.getDistance() < 213 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(-0.1);
			intakeRight.set(0.1);
			// Elevator
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			
			Timer.delay(0.02);
		}

		// Go Forward
		encReset();
		while (encAvg() < 33 && isAutonomous()) {
			myRobot.arcadeDrive(-0.6, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Shoot
		t.reset();
		t.start();
		while(t.get() < 0.3 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.24);
			intakeRight.set(-0.24);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		// Go Back
		encReset();
		while (encAvg() > -27 && isAutonomous()) {
			myRobot.arcadeDrive(0.7, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Set Positions
		double tempPos = elevatorEnc.getDistance();
		while (!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(tempPos, 0, 0.7, elevatorEnc.getDistance()) + 0.12);
			else
				elevatorPos.set(0);
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 95) < 4)
				intakePos.set(0.18);
			else
				intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		
		// Go Forward
		encReset();
		while (encAvg() < 5 && isAutonomous()) {
			myRobot.arcadeDrive(-0.75, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		stopRobot(0.1, true);

		// Turn to Box
		gyro.reset();
		while (gyro.getAngle() > -137 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.85);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
		}
		
		while (gyro.getAngle() < -137 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.8);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
		}
		myRobot.stopMotor();
		stopRobot(0.1, true);
		
		// Go Forward and Take the Power Cube
		encReset();
		while (encAvg() < 70 && isAutonomous()) {
			myRobot.arcadeDrive(-0.8, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		t.reset();
		t.start();
		while (t.get() < 0.8 && isAutonomous()) {
			myRobot.arcadeDrive(-0.5, 0);
			intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			intakeLeft.set(-0.6);
			intakeRight.set(0.6);
		}
		myRobot.stopMotor();
		
		gyro.reset();
		encReset();
		
		//go back
		
		while (encAvg() > -40) {
			intakeLeft.set(-0.42);
			intakeRight.set(0.42);
			myRobot.arcadeDrive(0.45, 0);
			intakePos.set((85 - intakeEnc.getDistance()) * 0.04);
		}
		intakeLeft.set(-0.15);
		intakeRight.set(0.15);
		
		while (Math.abs(intakeEnc.getDistance() - 45) > 3 ) {
			myRobot.stopMotor();
			//Lift
			if(elevatorEnc.getDistance() < 65 && !isParabolicFinished) {
				if (elevatorEnc.getDistance() - 0 <= 65 - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(0, 65, 0.45, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(0, 65, 0.4, elevatorEnc.getDistance()) - 0.17);
			} 
			//Stay stable
			else {
				elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
				isParabolicFinished = true;
			}
			
			//Intake stuff
			if (Math.abs(intakeEnc.getDistance() - 45) > 3) {
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.1658 : -0.3);
			}
			// Intake sabit tutma
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 ) {
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			}
		}
		
		t.start();
		while (t.get() < 1.25 && isAutonomous()) {
			myRobot.arcadeDrive(-0.45, 0.05);
			elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}

		t.reset();
		t.start();
		//Shoot the box
		while(t.get() < 0.5 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.28);
			intakeRight.set(-0.28);
			elevatorPos.set((elevatorEnc.getDistance() - 65) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		
		intakeLeft.set(0);
		intakeRight.set(0);
		
		//Drive back
		encReset();
		while(leftEnc.getDistance() > -30 && isAutonomous()) {
			myRobot.arcadeDrive(0.5, 0);
			if(intakeEnc.getDistance() > 3) {
				intakePos.set(-0.25);
			}
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Close intake
		while(intakeEnc.getDistance() > 3 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.25);
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		
		//Elevator Down
		while(!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set(parabolicControl(65, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			Timer.delay(0.02);
		}
		
		elevatorPos.set(0);
		intakePos.set(0);
		myRobot.stopMotor();
	}

	private void leftScaleSwamp() {
		Timer t = new Timer();
		t.reset();
		intakeLeft.set(-0.12);
		intakeRight.set(0.12);
		
		// Go Forward
		encReset();
		while (encAvg() < 350 && isAutonomous()) {
			myRobot.arcadeDrive(-1, 0);
			intakePos.set(-0.2);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakePos.set(0);
				intakeCalibration = false;
			}
			if(intakeTimer.get() > 0.5) {
				intakePos.set(0);
				intakeCalibration = false;
			}
			Timer.delay(0.02);
		}
		while (encAvg() < 570 && isAutonomous()) {
			myRobot.arcadeDrive(encAvg() * 0.001 - 1.27, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		
		stopRobot(0.1, true);

		//Gyro turn for first box, lift elevator
		while (gyro.getAngle() < 33 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.75);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);

		}
		myRobot.stopMotor();
		
		// Set Positions - Failsafe
		while (elevatorEnc.getDistance() < 213 && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 60) > 3)
				intakePos.set(intakeEnc.getDistance() - 60 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 60) <= 4 )
				intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			
			Timer.delay(0.02);
		}

		// Go Forward to Scale
		encReset();
		while (encAvg() < 33 && isAutonomous()) {
			myRobot.arcadeDrive(-0.625, 0);
			if (Math.abs(elevatorEnc.getDistance() - 215) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 215 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 215, 1.5, elevatorEnc.getDistance()) - 0.32);
				else
					elevatorPos.set(parabolicControl(0, 215, 1.6, elevatorEnc.getDistance()) - 0.12);
			}
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Shoot
		t.reset();
		t.start();
		while(t.get() < 0.3 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.27);
			intakeRight.set(-0.27);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		// Go Back
		encReset();
		while (encAvg() > -27 && isAutonomous()) {
			myRobot.arcadeDrive(0.7, 0.17);
			elevatorPos.set((elevatorEnc.getDistance() - 215) * 0.055);
			intakePos.set((60 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		
		// Go Back Totally
		encReset();
		double tempPos = elevatorEnc.getDistance();
		while (encAvg() > -250 && isAutonomous()) {
			myRobot.arcadeDrive(0.8, 0.145);
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(tempPos, 0, 0.7, elevatorEnc.getDistance()) + 0.12);
			else
				elevatorPos.set(0);
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 95) < 4)
				intakePos.set(0.18);
			else
				intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
		
		// Set Positions - Failsafe
		tempPos = elevatorEnc.getDistance();
		while (!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			// Elevator
			if (!isAtFloor())
				elevatorPos.set(parabolicControl(tempPos, 0, 0.7, elevatorEnc.getDistance()) + 0.12);
			else
				elevatorPos.set(0);
			
			// Intake
			if (Math.abs(intakeEnc.getDistance() - 95) < 4)
				intakePos.set(0.18);
			else
				intakePos.set((95 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}
		myRobot.stopMotor();
	}

	private void leftSwitchSwamp() {
		Timer autoTimer = new Timer();
		autoTimer.reset();
		
		// Go Forward
		encReset();
		while (encAvg() < 325 && isAutonomous()) {
			myRobot.arcadeDrive(0.001 * encAvg() - 0.95 , 0);
			Timer.delay(0.02);
		}

		//Gyro turn for switch
		while (gyro.getAngle() < 95 && isAutonomous()) {
			myRobot.arcadeDrive(0, 0.75);
			
			//Elevator
			if (Math.abs(elevatorEnc.getDistance() - 60) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 60 - elevatorEnc.getDistance())
					elevatorPos.set(parabolicControl(0, 60, 0.55, elevatorEnc.getDistance()) - 0.2);
				else
					elevatorPos.set(parabolicControl(0, 60, 0.5, elevatorEnc.getDistance()) - 0.17);
			}

			// Intake
			if (Math.abs(intakeEnc.getDistance() - 45) > 3)
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 )
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04);

		}
		
		myRobot.stopMotor();
		while (gyro.getAngle() > 95 && isAutonomous()) {
			myRobot.arcadeDrive(0, -0.7);
			//Elevator
			if (Math.abs(elevatorEnc.getDistance() - 60) < 3)
				elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			else {
				if (elevatorEnc.getDistance() - 0 <= 60 - elevatorEnc.getDistance()) {
					elevatorPos.set(parabolicControl(0, 60, 0.55, elevatorEnc.getDistance()) - 0.2);	
				}
				else
					elevatorPos.set(parabolicControl(0, 60, 0.5, elevatorEnc.getDistance()) - 0.17);
			}

			// Intake
			if (Math.abs(intakeEnc.getDistance() - 45) > 3)
				intakePos.set(intakeEnc.getDistance() - 45 < 0 ? 0.18 : -0.375);
			else if (Math.abs(intakeEnc.getDistance() - 45) <= 4 )
				intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
		}
		
		//Forward
		autoTimer.reset();
		autoTimer.start();
		while (autoTimer.get() < 1.25 && isAutonomous()) {
			myRobot.arcadeDrive(-0.4, 0);
			elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.055);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04);
			Timer.delay(0.02);
		}

		autoTimer.reset();
		autoTimer.start();
		
		//Shoot the box
		while(autoTimer.get() < 0.7 && isAutonomous()) {
			myRobot.stopMotor();
			intakeLeft.set(0.25);
			intakeRight.set(-0.25);
			elevatorPos.set((elevatorEnc.getDistance() - 60) * 0.05);
			intakePos.set((45 - intakeEnc.getDistance()) * 0.04); 
			Timer.delay(0.02);
		}
		intakeLeft.set(0);
		intakeRight.set(0);
		
		//Drive back
		encReset();
		while(leftEnc.getDistance() > -33 && isAutonomous()) {
			myRobot.arcadeDrive(0.45, 0);
			if(intakeEnc.getDistance() > 5) {
				intakePos.set(-0.25);
			}
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Close intake
		while(intakeEnc.getDistance() > 3 && isAutonomous()) {
			myRobot.stopMotor();
			intakePos.set(-0.2);
			if(!isAtFloor()) {
				elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			}
			Timer.delay(0.02);
		}
		
		//Elevator Down
		while(!isAtFloor() && isAutonomous()) {
			myRobot.stopMotor();
			elevatorPos.set(parabolicControl(60, 0, 0.25, elevatorEnc.getDistance()) + 0.07);
			Timer.delay(0.02);
		}
		
		elevatorPos.set(0);
		intakePos.set(0);
		myRobot.stopMotor();
	}
	
	private void stopRobot(double secs, boolean intakeStabilize) {
		double angle = intakeEnc.getDistance();
		Timer t = new Timer();
		t.reset();
		t.start();
		myRobot.stopMotor();
		while (t.get() < secs) {
			if (intakeStabilize)
				intakePos.set((angle - intakeEnc.getDistance()) * 0.04);
		}
		intakePos.stopMotor();
		myRobot.stopMotor();
	}

	private double encAvg() {
		return (leftEnc.getDistance() + rightEnc.getDistance()) / 2;
	}

	private void encReset() {
		leftEnc.reset();
		rightEnc.reset();
	}

	private void gyroTurn(double degrees, double turnSpeed) {
		gyro.reset();
		if (degrees > 0) {
			while (gyro.getAngle() < degrees && isAutonomous()) {
				myRobot.arcadeDrive(0, turnSpeed);
			}
			myRobot.stopMotor();
			while(gyro.getAngle() > degrees && isAutonomous())
				myRobot.arcadeDrive(0, -turnSpeed + 0.05);
		}
		else {
			while (gyro.getAngle() > degrees && isAutonomous()) {
				myRobot.arcadeDrive(0, -turnSpeed);
			}
			myRobot.stopMotor();
			while(gyro.getAngle() < degrees && isAutonomous())
				myRobot.arcadeDrive(0, turnSpeed - 0.05);
		}
		myRobot.stopMotor();
	}

	private void encTurn(double cm, double speed, boolean isLeft) {
		encReset();
		if (isLeft) {
			while (rightEnc.getDistance() < cm)
				myRobot.arcadeDrive(0,-speed);
			myRobot.stopMotor();
			while (rightEnc.getDistance() > cm)
				myRobot.arcadeDrive(0,speed);
		}
		else {
			while (leftEnc.getDistance() < cm)
				myRobot.arcadeDrive(0, speed);
			myRobot.stopMotor();
			while (leftEnc.getDistance() > cm)
				myRobot.arcadeDrive(0, -speed);
		}
		myRobot.stopMotor();
		encReset();
	}
	
	private void intakeCalibration() {
		while (intakeCalibration) {
			myRobot.stopMotor();
			intakePos.set(-0.25);
			if (!intakeLimit.get()) {
				intakeEnc.reset();
				intakeCalibration = false;
			}
			if(intakeTimer.get() > 0.5)
				intakeCalibration = false;
		}
	}
}
