
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import static frc.robot.Constants.HopperConstants.KEL_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.GUS_LIMIT_SWITCH; 
import static frc.robot.Constants.HopperConstants.CAN_ID_Hopper_Axle;

public class HopperSubsystem extends SubsystemBase implements Loggable {

    WPI_TalonSRX _HopperAxle = new WPI_TalonSRX(CAN_ID_Hopper_Axle);

     @Log(name = "Hopper High Limit")
     DigitalInput _HighSwitch=new DigitalInput(KEL_LIMIT_SWITCH); 
     
     @Log(name = "Hopper Low Limit")
     DigitalInput _LowSwitch=new DigitalInput(GUS_LIMIT_SWITCH); 
     
    public boolean isHighSwitchSet() {
        return !_HighSwitch.get();
    }
   
    public boolean isLowSwitchSet() {
        return !_LowSwitch.get();
    }

    public void HopperMotor(double hopper_spd){
       
        // temporary max speed
        double spd = -hopper_spd;

        if (Math.abs(spd) > 0.4){
            spd = 0.4*Math.signum(spd);  
        } 

        // temporary deadband
        if (Math.abs(spd) < 0.1){
            spd = 0;
        }
   
        _HopperAxle.set(spd); 
        
        //Logic needs to be tested to verify polarity is correct
        
        /*
        if ((_LowSwitch.get() && spd < 0) || _HighSwitch.get() && spd > 0 ) {
            _HopperAxle.set(spd);
        } else {
            _HopperAxle.set(0);
        }
        */
        

    }
}
