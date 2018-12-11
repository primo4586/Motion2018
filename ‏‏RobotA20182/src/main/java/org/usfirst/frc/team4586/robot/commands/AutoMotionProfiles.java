/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4586.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoMotionProfiles extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoMotionProfiles() {
    
    addSequential(new Motion2D(2.5));
    /*addSequential(new LiftToSwitch());
    addSequential(new Wait(1));
    addSequential(new CatchCube());
    addSequential(new Wait(0.5));
    addParallel(new AutoDriveTime(1.5));
    addSequential(new CatchCube());
    addSequential(new LiftToFloor());*/
    addSequential(new AutoTurn(90));
    /*addSequential(new CatchCube());
    addSequential(new AlignByVision());
    addSequential(new Wait(0.5));
    addParallel(new LiftToSwitch());
    addSequential(new CatchCube());
    addSequential(new Wait(0.5));
    addParallel(new AutoDriveTime(0.7));
    addSequential(new Wait(1));
    addSequential(new AutoTurn(90));
    addParallel(new MotionDriveStraight(1.3));
    addSequential(new Wait(1));
    addSequential(new CatchCube());*/
    
    
  }
}
