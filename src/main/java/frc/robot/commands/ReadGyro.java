/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UsbSerial;
import edu.wpi.first.wpilibj.SerialPort;
 
public class ReadGyro extends CommandBase {
   private UsbSerial arduino = new UsbSerial();
   SerialPort _serialport;
  /**

   */
  //, SerialPort ArduinoPort
  public ReadGyro(UsbSerial subsystem, SerialPort ArduinoPort) {
    _serialport = ArduinoPort;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    _serialport.reset();
    //_serialport.flush();
  }

  @Override
  public void execute() {
    //System.out.println("h");
    //_serialport
    arduino.getArduino(_serialport);
  }
  @Override
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
