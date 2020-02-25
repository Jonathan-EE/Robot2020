/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.HopperSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.RobotContainer;

public class HopperGyro extends PIDCommand implements Loggable  {
  private static double HopperP = 0.025;
  private static double HopperI = 0.001;
  private static double HopperD = 0.001;
  private static double angleTolerance = 0.5;
  private static double angleToleranceDPS = 5;
  @Log
  private static double setpoint = 25;
  /**
   * Creates a new Hopper control command with Gyro feedback.
   *
   * @param HopperSubsystem The drive subsystem this command will run on.
   * @param targetAngle The control input for driving forwards/backwards
   */
  public HopperGyro(HopperSubsystem hopper, DoubleSupplier targetAngleDegrees) {
    super(
        new PIDController(HopperP, HopperI, HopperD),
        () -> RobotContainer.arduino.getAngle(), // PV
        targetAngleDegrees.getAsDouble(), // Setpoint
        output -> hopper.HopperMotor(-output), // Pipe output to turn robot
        hopper// Require the hopper
        ); 
        setpoint = targetAngleDegrees.getAsDouble();
        System.out.printf("set %f",setpoint);

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
    //return getController().atSetpoint();
    return false;
  }
}
