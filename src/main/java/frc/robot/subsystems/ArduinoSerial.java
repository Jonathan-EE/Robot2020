
package frc.robot.subsystems;

import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.lib.CRC16CCITT;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ArduinoSerial extends SubsystemBase implements Loggable{
    //SerialPort ArduinoSerial = new SerialPort(115200, ArduinoPort);
    byte[] attitude  = new byte[0];
    byte[] slice = new byte[4];
    byte[] sliceCRC = new byte[2];
    int bytesGot;
    int crc;
    SerialPort arduino;
    //private final int frameHeaderByte = 0x59;

    @Log 
    private float angle = 999;

    private float angleTemp = 999;

    public ArduinoSerial(SerialPort serial){
        arduino = serial;
        arduino.reset();
        arduino.setReadBufferSize(20);
    }

    public void updateAngle(){
        
        bytesGot = arduino.getBytesReceived();
        bytesGot = testAttitude().length;
        if (bytesGot >= 10){
            //attitude = arduino.read(bytesGot);
            attitude = testAttitude();

            for (int i=0; i<attitude.length; i++){
                if (attitude[i] == 0x59 && attitude[i+1] == 0x42){
                    if (i+6 < attitude.length){
                        slice = Arrays.copyOfRange(attitude, i+2, i+6);
                        sliceCRC = Arrays.copyOfRange(attitude,i+6,i+8);
                        printByteArray(sliceCRC);
                        crc = CRC16CCITT.getCRC16(slice);
                        //if (Byte.compare(crc,sliceCRC) == 0 || 1){
                            angleTemp = ByteBuffer.wrap(slice).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                            if (Math.abs(angleTemp) < 360.1){
                                angle = angleTemp;
                            }
                        //}
                    }
                    continue;
                }
            }
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

    public double getAngle(){
        return angle;
    }

    private void printByteArray(byte[] byteArray){
        for (int i =0 ; i<byteArray.length; i++){
            System.out.printf("%x",byteArray[i]);
            if (i % 4 == 0){
                System.out.printf(" ");
            }
        }
        System.out.printf("\n");      
    }

    private static byte[] testAttitude(){
        byte[] attitude  = new byte[14];
        attitude[0] = (byte) 0x21;
        attitude[1] = (byte) 0x59;
        attitude[2] = (byte) 0x42;
        attitude[3] = (byte) 0x2D;
        attitude[4] = (byte) 0xB2;
        attitude[5] = (byte) 0x4D;
        attitude[6] = (byte) 0x40;
        attitude[7] = (byte) 0x00;
        attitude[8] = (byte) 0x00;
        attitude[9] = (byte) 0x00;
        attitude[10] = (byte) 0x00;
        attitude[11] = (byte) 0x00;
        attitude[12] = (byte) 0x00;
        attitude[13] = (byte) 0x00;
    
        return attitude;
    }
}