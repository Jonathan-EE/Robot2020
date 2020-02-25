/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.HopperSubsystem;

public class HopperGyroToggle extends CommandBase {
  private final HopperSubsystem m_hopper;

  /**

   */
  public HopperGyroToggle(HopperSubsystem subsystem) {
    m_hopper = subsystem;
    addRequirements(m_hopper);
  }

  @Override
  public void initialize() {
    m_hopper.toggleHopperSetpoint();
  }

  @Override
  public void execute() {
 
  }

  @Override
  public void end(boolean interrupted) {
 
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
