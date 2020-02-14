
package frc.robot.subsystems;

import java.lang.String;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.lib.CR16;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArduinoSerial extends SubsystemBase implements Loggable{
    //SerialPort ArduinoSerial = new SerialPort(115200, ArduinoPort);
    byte[] attitude  = new byte[0];
    byte[] slice = new byte[4];
    int bytesGot;
   SerialPort arduino;
    //private final int frameHeaderByte = 0x59;
    private float angle = 999;
    private float angleTemp = 999;
    //byte[] buffer = new byte[7];
    //int crc;

    public ArduinoSerial(SerialPort serial){
        arduino = serial;
        arduino.setReadBufferSize(20);
        //new Thread(this).start();
    }

    public float getAngle(){
        
        bytesGot = arduino.getBytesReceived();
        if (bytesGot >= 10){
            attitude = arduino.read(bytesGot);

            for (int i=0; i<attitude.length; i++){
                if (attitude[i] == 0x59 && attitude[i+1] == 0x59){
                    if (i+6 < attitude.length){
                        slice = Arrays.copyOfRange(attitude, i+2, i+6);
                        angleTemp = ByteBuffer.wrap(slice).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                        if (Math.abs(angleTemp) < 360.1){
                            angle = angleTemp;
                        }
                    }
                    continue;
                }
            }
        }
    
        return angle;

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