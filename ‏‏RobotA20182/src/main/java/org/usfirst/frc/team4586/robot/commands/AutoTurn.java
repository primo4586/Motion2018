package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.subsystems.Driver;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class AutoTurn extends Command {
	public Driver driver;
	double setPoint;
	double kP;
	double prevError;
	double kD;

    public AutoTurn(double setPoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	driver = Robot.driver;
    	this.setPoint = setPoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//setPoint = SmartDashboard.getNumber("GYRO Setpoint",0);
    	setTimeout(1);
    	driver.setGyroControllerSetPoint(setPoint);
    	driver.gyroController.enable();
    	driver.resetGyro();
    	kP = 0.11;
    	kD = 0.13;
    	prevError = 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double error = setPoint - driver.getGyro();
    	double der = (error - prevError) / 0.2;
    	double drcw = kD*der;
    	double prcw = kP*error;
    	if (Math.abs(driver.getGyro() - setPoint) >= 1) 
    		driver.arcadeDrive(0.5,(prcw + drcw));
    	else
    		driver.arcadeDrive(0.0, 0);
    	prevError = error;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	  return isTimedOut();
    }

    // Called once after isFinished returns true
    protected void end() {
    	driver.gyroController.disable();
    	driver.arcadeDrive(0, 0);
    	System.out.println(setPoint);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	driver.gyroController.disable();
    	driver.arcadeDrive(0, 0);
    }
}