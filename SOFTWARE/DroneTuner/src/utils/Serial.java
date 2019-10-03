/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package utils;

import com.fazecast.jSerialComm.SerialPort;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Asafe
 */
public class Serial {

    public static final String TAB = "\t";
    public static final String NEW_LINE = "\n";

    public static final String END = NEW_LINE;

    //   Arduino  --->   Computer
    public static final String YAW_KP = "YP";
    public static final String YAW_KI = "YI";
    public static final String YAW_KD = "YD";
    public static final String PITCH_KP = "PP";
    public static final String PITCH_KI = "PI";
    public static final String PITCH_KD = "PD";
    public static final String ROLL_KP = "RP";
    public static final String ROLL_KI = "RI";
    public static final String ROLL_KD = "RD";

    //   Computer  --->   Arduino
    public static final String YAW_INPUT = "YI";
    public static final String YAW_SETPOINT = "YS";
    public static final String PITCH_INPUT = "PI";
    public static final String PITCH_SETPOINT = "PS";
    public static final String ROLL_INPUT = "RI";
    public static final String ROLL_SETPOINT = "RS";
    public static final String M1 = "A";
    public static final String M2 = "B";
    public static final String M3 = "C";
    public static final String M4 = "D";

    private SerialPort serialPort;
    private InputStream inputStream;
    private OutputStream outputStream;

    private Thread threadListener;

    private DataListener dataListener;
    
    private String parameter = "", data = "";
    private boolean enableListener;

    public Serial() {
    }

    public void setSerialPort(SerialPort commPort) {
        serialPort = commPort;
        inputStream = serialPort.getInputStream();
        outputStream = serialPort.getOutputStream();
    }

    public void initListener(DataListener dataListener) {

        this.dataListener = dataListener;
        this.enableListener = true;
        parameter = "";
        data = "";

        threadListener = new Thread(() -> {
            
            while(enableListener){
                
                while (serialPort.bytesAvailable() < 1){  if(!enableListener) break;  }
                if(!enableListener) break;
                
                byte[] readBuffer = new byte[serialPort.bytesAvailable()];
                serialPort.readBytes(readBuffer, readBuffer.length);
                
                for (int i = 0; i < readBuffer.length; i++) {
                    char inChar = (char) readBuffer[i];
                    
                    if (Character.toString(inChar).equals(Serial.END)) {
                        
                        float value = 0;
                        
                        try {
                            value = Float.parseFloat(data);
                        } catch (NumberFormatException e) {
                            System.err.print("[utils.Serial] - ERROR Format Number");
                            System.err.println("\tData: " + data);
                        }
                        
                        if (dataListener != null) {
                            dataListener.onDataListener(parameter, value);
                        }
                        
                        parameter = "";
                        data = "";
                    } else {
                        if (Character.isLetter(inChar)) {
                            parameter += inChar;
                        } else {
                            data += inChar;
                        }
                    }
                }
            }
            
        });

        threadListener.setPriority(Thread.MAX_PRIORITY);
        threadListener.start();
    }

    public synchronized void stopListener() {
        enableListener = false;
        parameter = "";
        data = "";
    }

    public synchronized boolean writeData(String data) {
        try {
            outputStream.write(data.getBytes());
            return true;
        } catch (IOException|NullPointerException e) {
            return false;
        }
    }

    public synchronized boolean writeData(byte data) {
        try {
            outputStream.write(data);
            return true;
        } catch (IOException|NullPointerException e) {
            return false;
        }
    }

    public synchronized boolean writeData(int data) {
        try {
            outputStream.write(data);
            return true;
        } catch (IOException|NullPointerException e) {
            return false;
        }
    }

    public interface DataListener {

        public void onDataListener(String parameter, float value);
    }

}
