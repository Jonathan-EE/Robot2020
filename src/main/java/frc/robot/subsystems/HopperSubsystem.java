
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.RobotContainer;

import static frc.robot.Constants.HopperConstants.KEL_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.GUS_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;

public class HopperSubsystem extends ProfiledPIDSubsystem {
    private static double HopperP = 0.025;
    private static double HopperI = 0.001;
    private static double HopperD = 0.001;
    private static double angleTolerance = 0.5;
    private static double angleToleranceDPS = 5;
    
    private static double kMaxTurnRateDegPerS = 1;
    private static double kMaxTurnAccelerationDegPerSSquared = 0.2;

    private static double hopperSetpoint = 25;

    WPI_TalonSRX _HopperAxle = new WPI_TalonSRX(CAN_ID_Hopper_Axle);
    //private final ArmFeedforward m_feedforward =
    //  new ArmFeedforward(0, 0,
    //                     ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

     @Log(name = "Hopper High Limit")
     DigitalInput _HighSwitch = new DigitalInput(KEL_LIMIT_SWITCH); 
     
     @Log(name = "Hopper Low Limit")
     DigitalInput _LowSwitch = new DigitalInput(GUS_LIMIT_SWITCH); 
     
    public boolean isHighSwitchSet() {
        return !_HighSwitch.get();
    }
   
    public boolean isLowSwitchSet() {
        return !_LowSwitch.get();
    }

    public HopperSubsystem() {

        super(new ProfiledPIDController(HopperP, HopperI, HopperD, new TrapezoidProfile.Constraints(
            kMaxTurnRateDegPerS, kMaxTurnAccelerationDegPerSSquared)), 0);

        setGoal(hopperSetpoint);
        //m_encoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
        // Start arm at rest in neutral position
       
      }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
      // Calculate the feedforward from the sepoint
      //double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
      // Add the feedforward to the PID output to get the motor output
      _HopperAxle.setVoltage(output);
    }

    @Override
    public double getMeasurement() {
      return RobotContainer.arduino.getAngle();
    }

    public void HopperMotor(double hopper_spd){
       
        // temporary max speed
        double spd = -hopper_spd;

        if (Math.abs(spd) > 0.4){
            spd = 0.4*Math.signum(spd);  
        } 

        // temporary deadband
        if (Math.abs(spd) < 0.01){
            spd = 0;
        }
   
        //_HopperAxle.set(spd); 
        
        //Logic needs to be tested to verify polarity is correct  
        if ((_LowSwitch.get() && spd < 0) || _HighSwitch.get() && spd > 0 ) {
            _HopperAxle.set(spd);
        } else {
            _HopperAxle.set(0);
        }

    }
}
