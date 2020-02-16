/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.HopperSubsystem;

import frc.robot.RobotContainer;

public class HopperGyro extends PIDCommand  {
  private static double HopperP = 0.1;
  private static double HopperI = 0.0;
  private static double HopperD = 0.0;
  private static double angleTolerance = 3;
  private static double angleToleranceDPS = 1;
  /**

   */
  public HopperGyro(double targetAngleDegrees, HopperSubsystem hopper) {
    super(
        new PIDController(HopperP, HopperI, HopperD),
        () -> RobotContainer.arduino.getAngle(), // PV
        targetAngleDegrees, // Setpoint
        output -> hopper.HopperMotor(output), // Pipe output to turn robot
        hopper// Require the hopper
        ); 

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(angleTolerance, angleToleranceDPS);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
