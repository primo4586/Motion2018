package org.usfirst.frc.team4586.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Climber extends Subsystem {
	Compressor compressor;
	WPI_TalonSRX climbMotor1;
	WPI_TalonSRX climbMotor2;
	Solenoid openPlatform;
	Solenoid closePlatform;
	Solenoid openShloplopSolenoid;
	Solenoid closeShloplopSolenoid;
	boolean isOpen;
	boolean toOpenShloplop;

	public Climber(WPI_TalonSRX climbMotor1, WPI_TalonSRX climbMotor2, Compressor compressor, Solenoid openPlatform, Solenoid closePlatform ,Solenoid openShloplopSolenoid ,Solenoid closeShloplopSolenoid) {
		this.isOpen = false;
		this.climbMotor1 = climbMotor1;
		this.climbMotor2 = climbMotor2;
		this.compressor = compressor;
		this.openPlatform = openPlatform;
		this.closePlatform = closePlatform;
		this.openShloplopSolenoid=openShloplopSolenoid;
		this.closeShloplopSolenoid=closeShloplopSolenoid;
		
	}

    // checks if the platforms' pistons are opened
    public boolean isOpened() {
	return openPlatform.get();
    }

    // set the pistons state
    public void setPiston(boolean isOpened) {
	openPlatform.set(isOpened);
	closePlatform.set(!isOpened);
    }

    public void setSpeedClimb(double speed) {
	this.climbMotor1.set(speed);
	this.climbMotor2.set(speed);
    }
    
    public void setSpeedClimbL(double speed) {
    	this.climbMotor1.set(speed);
    }
    
    public void setSpeedClimbR(double speed) {
    	this.climbMotor2.set(speed);
    }

    public void setPlatform(boolean open) {
    	openPlatform.set(open);
    	closePlatform.set(!open);
    }

    public void stopAllClimberMotors() {
	this.climbMotor1.set(0);
	this.climbMotor2.set(0);
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
	// Set the default command for a subsystem here.
	// setDefaultCommand(new MySpecialCommand());
    }
    public void setShloplop(boolean toOpenShloplop)
    {
    	this.openShloplopSolenoid.set(toOpenShloplop);
    	this.closeShloplopSolenoid.set(!toOpenShloplop);
    }
    public boolean isOpendShloplop()
    {
    	return openShloplopSolenoid.get();
    }
    
    
}
