
package frc.robot.subsystems;

import java.lang.String;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
//SerialPort ArduinoSerial = new SerialPort(115200, ArduinoPort);
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.lib.CR16;

public class UsbSerial extends SubsystemBase {

    //SerialPort ArduinoSerial = new SerialPort(115200, ArduinoPort);
    byte[] attitude  = new byte[4];
    int bytesGot;
    float pitch;
   
    //
    public void getArduino(SerialPort ArduinoSerial){
        //attitude = ArduinoSerial.readString();

        bytesGot = ArduinoSerial.getBytesReceived();
        if (bytesGot > 4){
            attitude = ArduinoSerial.read(4);
            pitch = ByteBuffer.wrap(attitude).order(ByteOrder.BIG_ENDIAN).getFloat();
            System.out.printf("%f\n",pitch);
    
           
            for (int i=0; i<4; i++){
                System.out.printf("%x",attitude[i]);
            }
            System.out.printf("\n");
            
        }
        
        //System.out.printf("%d",bytesGot);
        //System.out.printf("\n");

        /*
        // test code for arduino crc. 
        buffer[0] = 0x21;
        buffer[1] = 0x04;//Function Code; in this case 0x04 is the read command
        buffer[2] = 0;
        buffer[3] = 20;//Starting address of the register
        buffer[4] = 0;
        buffer[5] = 10;//Number of registers to read
        
        // ArduinoSerial.readString();
        crc = CR16.getCRC16(buffer);
        System.out.printf("%x \n", crc); 
        */
    }
}