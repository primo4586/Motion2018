package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.OI;
import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.subsystems.CubeSystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LiftToSwitch extends Command {

	CubeSystem cubeSystem;
	OI oi;
	double speedMol;
	boolean ok;

	public LiftToSwitch() {
		this.cubeSystem = Robot.cubeSystem;
		this.oi = Robot.m_oi;
		this.speedMol = -1;
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(4);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (cubeSystem.getFloorSensor())
			speedMol = 1;
		cubeSystem.setSpeedElevators(speedMol * SmartDashboard.getNumber("Elevator Speed", 0));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return cubeSystem.getSwitchSensor()||isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		cubeSystem.setSpeedElevators(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		cubeSystem.setSpeedElevators(0);
	}
}
