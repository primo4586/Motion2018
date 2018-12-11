package org.usfirst.frc.team4586.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class CubeSystem extends Subsystem {
	Compressor compressor;
	Solenoid solenoid1;
	Solenoid solenoid2;
	Solenoid pushCubeOpen;
	Solenoid pushCubeClose;

	WPI_TalonSRX elevatorsMotor;
	WPI_TalonSRX elevatorMotor2;
	DigitalInput scaleSensor;
	DigitalInput switchSensor;
	DigitalInput floorSensor;
	boolean isCubeCatcherOpen;

	boolean canUseElevator;

	public CubeSystem(Solenoid solenoid2, Solenoid solenoid1, Solenoid pushCubeOpen, Solenoid pushCubeClose,
			Compressor compressor, WPI_TalonSRX elevatorsMotor, WPI_TalonSRX elevatorMotor2, DigitalInput scaleSensor, DigitalInput switchSensor,
			DigitalInput floorSensor) {
		this.compressor = compressor;
		this.solenoid1 = solenoid1;
		this.solenoid2 = solenoid2;
		this.pushCubeOpen = pushCubeOpen;
		this.pushCubeClose = pushCubeClose;
		this.elevatorsMotor = elevatorsMotor;
		this.elevatorMotor2 = elevatorMotor2;
		this.scaleSensor = scaleSensor;
		this.switchSensor = switchSensor;
		this.floorSensor = floorSensor;
		this.isCubeCatcherOpen = solenoid1.get();
		canUseElevator = true;
	}

	// checks if the pusher piston is opened
	public boolean isOpenedPusher() {
		return pushCubeOpen.get();
	}

	public void setCubePusher(boolean isOpenedPusher) {
		pushCubeOpen.set(isOpenedPusher);
		pushCubeClose.set(!isOpenedPusher);

	}

	// checks if the platforms' pistons are opened
	public boolean isOpened() {
		return this.solenoid1.get();
	}

	public void setIsCubeCatcherOpen(boolean value) {
		this.isCubeCatcherOpen = value;
	}
	//
	// // set the pistons state
	// public void setPiston1(boolean isOpened) {
	// if (isOpened) {
	// solenoid2.set(false);
	// } else {
	// solenoid1.set(true);
	// }
	// }
	//
	// public void setPiston2(boolean isOpened) {
	// if (isOpened) {
	// solenoid1.set(false);
	// } else {
	// solenoid2.set(true);
	// }
	// }

	public void setPistonR(boolean toOpen) {
		solenoid1.set(toOpen);
	}

	public void setPistonL(boolean toOpen) {
		solenoid2.set(toOpen);
	}

	// elevator's speed
	public void setSpeedElevators(double speed) {
		if (this.canUseElevator) {
			this.elevatorsMotor.set(speed);
			this.elevatorMotor2.set(speed);
		}
		else {
			this.elevatorsMotor.set(0);
			this.elevatorMotor2.set(0);
		}
	}

	public void setCanUseElevator(boolean canUseElevator) {
		this.canUseElevator = canUseElevator;
	}

	public double getSpeedElevators() {
		return this.elevatorsMotor.get();
	}

	// sensors
	public boolean getFloorSensor() {
		return !floorSensor.get();
	}

	public boolean getSwitchSensor() {
		return !switchSensor.get();
	}

	public boolean getScaleSensor() {
		return !scaleSensor.get();
	}

	public void stopElevators() {
		this.elevatorsMotor.set(0);
	}
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
