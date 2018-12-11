package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.subsystems.CubeSystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LiftByTime extends Command {

	private CubeSystem cubeSystem;
	private double time;
	private boolean up;

	public LiftByTime(double _time, boolean _up) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.cubeSystem = Robot.cubeSystem;
		this.time = _time;
		this.up = _up;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(this.time);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (this.up) {
			this.cubeSystem.setSpeedElevators(SmartDashboard.getNumber("Elavator Speed", 1));
		} else {
			this.cubeSystem.setSpeedElevators(-SmartDashboard.getNumber("Elavator Speed", 1));
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		this.cubeSystem.stopElevators();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		this.cubeSystem.stopElevators();
	}
}
