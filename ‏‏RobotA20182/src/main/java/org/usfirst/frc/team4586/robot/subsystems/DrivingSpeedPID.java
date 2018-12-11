package org.usfirst.frc.team4586.robot.subsystems;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 */
public class DrivingSpeedPID implements PIDOutput {
	private double speed;
	
	public DrivingSpeedPID() {
		speed = 0;
	}

	public double getSpeed() {
		return speed;
	}

	public void updateSpeed(double speed) {
		this.speed = speed;
		
	}


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }


	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		speed = output;
	}
}

