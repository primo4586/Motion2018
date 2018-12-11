package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.subsystems.Driver;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AlignByVision extends Command {

	private Driver driver;
	private NetworkTable table;
	private double angle;
	private double distance;

	public AlignByVision() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.driver = Robot.driver;
		this.table = Robot.table;
  
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		//angle = table.getEntry("angle").getDouble(0);
		//this.distance = table.getEntry("distance").getDouble(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
    this.angle = this.table.getEntry("angle").getDouble(0); //Gets the angle from the table
		System.out.println(this.angle);
		if(this.angle > 14)
		{
			this.driver.arcadeDrive(0.0, 0.45);
		}
		else if(this.angle < -14)
		{
			this.driver.arcadeDrive(0.0, -0.45);
		}
    else if (this.angle > 6) 
    {
      this.driver.arcadeDrive(0.4, 0.4);
    }
    else if (this.angle < -6)
    {
      this.driver.arcadeDrive(0.4, -0.4);
    }
    else if (this.angle > -6 && this.angle < -3)
    {
      this.driver.arcadeDrive(0.7, -0.3);
    }
    else if (this.angle < 6 && this.angle > 3)
    {
      this.driver.arcadeDrive(0.7, 0.3);
    }
    else if(this.angle < 2 && this.angle > 0.25)
    {
      this.driver.arcadeDrive(0.8, 0);
    }
    else if(this.angle < -2 && this.angle > -0.25)
    {
      this.driver.arcadeDrive(0.8, 0);
    }
    else
    {
      System.out.println("drive straight");
      this.driver.arcadeDrive(0.8, 0.0);
		}
		
		
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(this.angle) < 2 && table.getEntry("distance").getDouble(0) < 30;
	}

	// Called once after isFinished returns true
	protected void end() {
		this.driver.stopAllWheels();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		this.end();
		System.out.println("AlignByVision interrupted");
	}
}
