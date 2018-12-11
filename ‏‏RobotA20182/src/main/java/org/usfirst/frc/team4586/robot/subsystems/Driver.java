package org.usfirst.frc.team4586.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 *
 */
public class Driver extends Subsystem {

	WPI_TalonSRX leftFrontMotor;
	WPI_TalonSRX leftBackMotor;
	WPI_TalonSRX rightFrontMotor;
	WPI_TalonSRX rightBackMotor;
	AnalogGyro gyro;
	Encoder encoder;
	SpeedControllerGroup rightController, leftController;
	DifferentialDrive diffDrive;

	private DrivingGyroPID gyroSource;
	private DrivingRotationPID rotationPID;
	public PIDController gyroController;
	
	private DrivingEncoderPID encoderSource;
	private DrivingSpeedPID speedPID;
	public PIDController encoderController;

	public Driver(WPI_TalonSRX leftFrontMotor, WPI_TalonSRX leftBackMotor, WPI_TalonSRX rightFrontMotor,
			WPI_TalonSRX rightBackMotor, AnalogGyro gyro, Encoder drivingEncoder) {
		this.leftFrontMotor = leftFrontMotor;
		this.leftBackMotor = leftBackMotor;
		this.rightFrontMotor = rightFrontMotor;
		this.rightBackMotor = rightBackMotor;
		this.gyro = gyro;
		this.encoder = drivingEncoder;
		encoder.setDistancePerPulse(0.0239);
		this.rightController = new SpeedControllerGroup(this.rightBackMotor, this.rightFrontMotor);
		this.leftController = new SpeedControllerGroup(this.leftBackMotor, this.leftFrontMotor);
		this.diffDrive = new DifferentialDrive(this.leftController, this.rightController);
		this.encoderSource = new DrivingEncoderPID(drivingEncoder);
		this.speedPID =  new DrivingSpeedPID();
		this.encoderController = new PIDController(0.025, 0, 0.01, this.encoderSource, this.speedPID);
		this.gyroSource = new DrivingGyroPID(this.gyro);
		this.rotationPID = new DrivingRotationPID();
		this.gyroController = new PIDController(-0.118, 0.0, 0.1, this.gyroSource, this.rotationPID); 
		encoderController.setAbsoluteTolerance(0.7);
		
		//PID Values - 
		//-0.035, 0.0, 0.01 - Gyro PID Turn
		//0.025, 0, 0.01 - Encoder PID Drive
		gyroController.setAbsoluteTolerance(1);
	}

	public PIDController getGyroController(){
		return this.gyroController;
	}
	
	public PIDController getEncoderController(){
		return this.encoderController;
	}
	
	public void setSetPointGyro(double setpoint) {
		this.gyroController.setSetpoint(setpoint);
	}
	
	public void enableGyro() {
		this.gyroController.enable();
	}
	
	public void enableEncoder() {
		this.encoderController.enable();
	}
	
	public void disableEncoder() {
		this.encoderController.disable();
	}
	
	
	public void disableGyro() {
		this.gyroController.disable();
	}
	
	public double getRotation() {
		return this.rotationPID.getRotation();
	}
	
	public double getSpeed() {
		return this.speedPID.getSpeed();
	}
	// wheels
	public double getWheelSpeedLeftFront() {
		return leftFrontMotor.get();
	}

	public double getWheelSpeedLeftBack() {
		return leftBackMotor.get();
	}

	public double getWheelSpeedRightBack() {
		return rightBackMotor.get();
	}

	public double getWheelSpeedRightFront() {
		return rightFrontMotor.get();
	}

	// stops the wheels
	public void stopAllWheels() {
		this.leftBackMotor.set(0);
		this.leftFrontMotor.set(0);
		this.rightFrontMotor.set(0);
		this.rightBackMotor.set(0);
	}

	// gyro
	public void resetGyro() {
		this.gyro.reset();
	}

	public double getGyroAngle() {
		return this.gyro.getAngle();
	}
	
	
	
	public double getPIDspeed() {
		return speedPID.getSpeed();
	}

	/*public double getPIDRotation() {
		return rotation.getRotation();
	}*/
	
	public double getPIDRotationInPlace() {
		return rotationPID.getRotation();
	}
	
	
	// calibrates the gyro
	public void calibrateGyro() {
		this.gyro.calibrate();
	}

	// encoder
	public double getSpeedEncoder() {
		return encoder.getRate();
	}

	public double getEncoderDistance() {
		return encoder.getDistance();
	}
	
	public double getEncoderValue() {
		return encoder.get();
	}
	
	public void resetEncoder() {
		this.encoder.reset();
	}

	// drive
	public void arcadeDrive(double speed, double rotation) {
		this.diffDrive.arcadeDrive(speed, rotation);
	}

	public void setGyroControllerSetPoint(double setPoint) {
		gyroController.setSetpoint(setPoint);
	}

	public void setEncoderControllerSetPoint(double setPoint) {
		encoderController.setSetpoint(setPoint);
	}

	public void setLeft(double speed) {
		this.leftFrontMotor.set(speed);
		this.leftBackMotor.set(speed);
	}

	public void setRight(double speed) {
		this.rightFrontMotor.set(-speed);
		this.rightBackMotor.set(-speed);
	}

	

	public double getGyro() {
		return (gyro.getAngle() % 360);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
