
package frc.robot.subsystems;

import java.lang.String;
import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.lib.CR16;

public class ArduinoSerial extends SubsystemBase{
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

	
	public void test()
	{

        int[] frame = new int[4];
        byte[] data = new byte[0];

		int frameIndex = 0;
        int dataIndex = 0;

        // Reset serial port to empty buffers;
        arduino.reset();

        while (true)
		{
			if (dataIndex >= data.length)
			{
				data = arduino.read(arduino.getBytesReceived());
				if (0 == data.length)
				{
					continue;
				}
                dataIndex = 0;
                for (int i = 0; i<data.length; i++){
                    System.out.printf("%x",data[i]);
                    if (i%4 == 0){
                        System.out.printf("\n");
                    }
                }
                
			}
            
            angle = ByteBuffer.wrap(data).order(ByteOrder.BIG_ENDIAN).getFloat();

            
            try{
                Thread.sleep(100);
            } catch(InterruptedException e) {
                System.out.println("SerialError");
            }
            
		}

    }
 
   
    //
    public float getAngle(){
        //System.out.printf("Test: %f\n", angle);
        //return angle;
        
        //attitude = ArduinoSerial.readString();

        
        bytesGot = arduino.getBytesReceived();
        if (bytesGot >= 10){
            attitude = arduino.read(bytesGot);
            //angle = ByteBuffer.wrap(attitude).order(ByteOrder.BIG_ENDIAN).getFloat();
            
            /*
            for (int i=0; i<6; i++){
                System.out.printf("%x",attitude[i]);
            }
            System.out.printf("\n");
            */

            for (int i=0; i<attitude.length; i++){
                if (attitude[i] == 0x59 && attitude[i+1] == 0x59){
                    if (i+6 < attitude.length){
                     slice = Arrays.copyOfRange(attitude, i+2, i+6);
                     /*
                     System.out.printf("slice: ");
                     for(int j=0;j<slice.length;j++){
                        System.out.printf("%x",slice[j]);
                     }
                     System.out.printf("\n");
                     */
                    
                     angleTemp = ByteBuffer.wrap(slice).order(ByteOrder.LITTLE_ENDIAN).getFloat();
                     if (Math.abs(angleTemp) < 361){
                         angle = angleTemp;
                     }
                    }
                    continue;
                }
            }

            
            //System.out.printf("%f\n",angle);
            /*
            for (int j=0; j<4; j++){
                System.out.printf("slice: %x",slice[j]);
            }
           */
            
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