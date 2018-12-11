/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4586.robot.commands;

import org.usfirst.frc.team4586.robot.Robot;
import org.usfirst.frc.team4586.robot.RobotMap;
import org.usfirst.frc.team4586.robot.subsystems.Driver;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class MotionDriveStraight extends Command {
  Trajectory trajectory;
	TankModifier tankModifier, modifer;
  EncoderFollower leftEncoderFollower, rightEncoderFollower;
  Driver driver;
  double distance;
  public MotionDriveStraight(double distance) {
    this.driver = Robot.driver;
    this.distance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //sets the path 
    Waypoint[] points = new Waypoint[] {
			new Waypoint(0, 0, 0),
			new Waypoint(this.distance, 0, 0)
		};
    
    //the config is the settings of MP and makes the trajectory
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 0.02, 2, 7.50, 90.0);
		trajectory = Pathfinder.generate(points, config);
		modifer = new TankModifier(trajectory).modify(0.6);
    RobotMap.leftBackMotor.setSelectedSensorPosition(0, 0, 10);
    RobotMap.rightFrontMotor.setSelectedSensorPosition(0, 0, 10);
    driver.setEncoderControllerSetPoint(2);
    driver.encoderController.enable();
    setTimeout(3);
    leftEncoderFollower = new EncoderFollower(modifer.getLeftTrajectory());
    rightEncoderFollower = new EncoderFollower(modifer.getRightTrajectory());	
    //sets the encoder pid and configure
		leftEncoderFollower.configureEncoder(RobotMap.leftBackMotor.getSelectedSensorPosition(0), 8162, 0.1524);
    rightEncoderFollower.configureEncoder(RobotMap.rightFrontMotor.getSelectedSensorPosition(0), 8162, 0.1524);
    	
    leftEncoderFollower.configurePIDVA(0.75, 0.0, 0.0, 1 / 2, 0);
    rightEncoderFollower.configurePIDVA(0.75, 0.0, 0.0, 1 / 2, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //calculates the speed of the motors
    double l = leftEncoderFollower.calculate(RobotMap.leftBackMotor.getSelectedSensorPosition(0));
    	double r = rightEncoderFollower.calculate(RobotMap.rightFrontMotor.getSelectedSensorPosition(0));
    	double gyro_heading = RobotMap.gyro.getAngle();    // Assuming the gyro is giving a value in degrees
		double desired_heading = Pathfinder.r2d(leftEncoderFollower.getHeading());  // Should also be in degrees

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		double turn = 0.8 * (-1.0/80.0) * angleDifference;
    	driver.setLeft(l);
    	driver.setRight(-r);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
