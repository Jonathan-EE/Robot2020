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

//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
//kllimport edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.HopperSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.RobotContainer;

public class HopperGyro extends ProfiledPIDCommand {
  private static HopperSubsystem m_hopper;

  private static double HopperP = 0.120;
  private static double HopperI = 0.001;
  private static double HopperD = 0.001;
  private static double angleTolerance = 1;
  private static double angleToleranceDPS = 1;

  private static double kMaxTurnRateDegPerS = 1;
  private static double kMaxTurnAccelerationDegPerSSquared = 0.2;

  /**
   * Creates a new Hopper control command with Gyro feedback.
   *
   * @param HopperSubsystem The drive subsystem this command will run on.
   * @param targetAngle The control input for driving forwards/backwards
   */
  public HopperGyro(HopperSubsystem hopper) {
    super(
          new ProfiledPIDController(HopperP, HopperI, HopperD, new TrapezoidProfile.Constraints(
            kMaxTurnRateDegPerS,
            kMaxTurnAccelerationDegPerSSquared)),
          () -> RobotContainer.arduino.getAngle(), // PV
          40, // Setpoint
          (output, setpoint) -> hopper.HopperMotor(-output), // Pipe output to turn robot
          hopper// Require the hopper
          ); 
         


    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    //getController().setTolerance(angleTolerance, angleToleranceDPS);
    m_hopper = hopper;
    addRequirements(hopper);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute(){
    getController().setGoal(m_hopper.getHopperSetpoint());
    System.out.printf("setpoint %f\n",getController().getSetpoint());
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
