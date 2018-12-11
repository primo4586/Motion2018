package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.subsystems.CubeSystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class CatchCube extends Command {
	CubeSystem cubeSystem;
	boolean isOpenedBothPistons;
	boolean isToOpen;

	public CatchCube() {
		this.cubeSystem = Robot.cubeSystem;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		this.isOpenedBothPistons = false;
		isToOpen = cubeSystem.isOpened();
		if (!isToOpen) {
			this.cubeSystem.setPistonR(true);

		} else {
			this.cubeSystem.setPistonL(false);
		}
		setTimeout(0.1);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (isTimedOut()) {
			if (!isToOpen) {
				this.cubeSystem.setPistonL(true);
			} else {
				this.cubeSystem.setPistonR(false);
			}
			this.isOpenedBothPistons = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isOpenedBothPistons;
	}

	// Called once after isFinished returns true
	protected void end() {
		this.cubeSystem.setIsCubeCatcherOpen(!isToOpen);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
