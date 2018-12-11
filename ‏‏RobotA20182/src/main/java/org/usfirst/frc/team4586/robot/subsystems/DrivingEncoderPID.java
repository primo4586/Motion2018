package org.usfirst.frc.team4586.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 *
 */
public class DrivingEncoderPID implements PIDSource {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	public Encoder encoder;
	
	public DrivingEncoderPID(Encoder encoder) {
		// TODO Auto-generated constructor stub
		this.encoder = encoder;
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return encoder.getDistance();
	}
}

