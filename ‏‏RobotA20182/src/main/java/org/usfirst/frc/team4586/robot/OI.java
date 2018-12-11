/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4586.robot;

import org.usfirst.frc.team4586.robot.commands.AlignByVision;

import org.usfirst.frc.team4586.robot.commands.AutoTurn;
import org.usfirst.frc.team4586.robot.commands.CalibrateGyro;
import org.usfirst.frc.team4586.robot.commands.CatchCube;
import org.usfirst.frc.team4586.robot.commands.CubeCatcherNoDelay;
import org.usfirst.frc.team4586.robot.commands.LiftToFloor;
import org.usfirst.frc.team4586.robot.commands.LiftToSwitch;
import org.usfirst.frc.team4586.robot.commands.Motion2D;
import org.usfirst.frc.team4586.robot.commands.MotionDriveStraight;
import org.usfirst.frc.team4586.robot.commands.SwitchCompressor;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	//driver
	public Joystick joystickDriver;
	public JoystickButton liftToFloorDriver;
	public JoystickButton catchCube;
	//public JoystickButton invert;
	public JoystickButton calibrateGyro;
	public JoystickButton visionAlign;
	public JoystickButton cubePusher;
	public JoystickButton noDelayCatch;
	public JoystickButton gyroTurn;
	public JoystickButton openBothThings;
	
	//operator
	public Joystick joystickOpertor;
	public JoystickButton climbL;
	public JoystickButton climbR;
	public JoystickButton liftToScale;
	public JoystickButton liftToFloor;
	public JoystickButton liftToSwitch;
	public JoystickButton openPlatform;
	public JoystickButton switchCompressor;
	public JoystickButton downRight;
	public JoystickButton toggleShloplop;
	public JoystickButton unlockElevator;
	
	
	public OI()
	{
		//buttons left -> 2,7
				joystickDriver = new Joystick(0);
				liftToFloorDriver = new JoystickButton(joystickDriver , 2); //B
				//invert = new JoystickButton(joystickDriver , 4); Y
				visionAlign = new JoystickButton(joystickDriver , 8); //BACK
				cubePusher = new JoystickButton(joystickDriver , 3);//X
				catchCube = new JoystickButton(joystickDriver , 5); //LB
				noDelayCatch = new JoystickButton(joystickDriver, 6); //RB
				openBothThings = new JoystickButton(joystickDriver, 1);//A
				

				joystickOpertor = new Joystick(1);
				liftToFloor = new JoystickButton(joystickOpertor , 3);
				//liftToSwitch = new JoystickButton(joystickOpertor ,7);
				climbR = new JoystickButton(joystickOpertor , 6);
				openPlatform = new JoystickButton(joystickOpertor , 2);
				switchCompressor = new JoystickButton(joystickOpertor, 5);
				calibrateGyro = new JoystickButton(joystickOpertor , 8);
				downRight = new JoystickButton(joystickOpertor, 4);
				//toggleShloplop = new JoystickButton(joystickOpertor, 1);
				unlockElevator = new JoystickButton(joystickOpertor, 10);
				gyroTurn = new JoystickButton(joystickOpertor, 1);
				
				//driver commands
				liftToFloorDriver.whenPressed(new Motion2D(3));
				//invert.whenPressed(new Invert());
				calibrateGyro.whenPressed(new CalibrateGyro());
				visionAlign.whileHeld(new MotionDriveStraight(SmartDashboard.getNumber("MotionDistance", 2)));
				catchCube.whenPressed(new CatchCube());
				noDelayCatch.whenPressed(new CubeCatcherNoDelay());
				openBothThings.whileHeld(new AlignByVision());
				
				//operator commands
				liftToFloor.toggleWhenPressed(new LiftToFloor());
				gyroTurn.toggleWhenPressed(new AutoTurn(90));
				switchCompressor.whenPressed(new SwitchCompressor());

	}
}