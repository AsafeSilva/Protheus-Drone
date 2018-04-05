/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package utils;

/**
 *
 * @author Asafe
 */
public class DroneMotors {
    
    public static final String M1_NAME = "Motor 1";
    public static final String M2_NAME = "Motor 2";
    public static final String M3_NAME = "Motor 3";
    public static final String M4_NAME = "Motor 4";
    
    public static final int M1_ID = 0;
    public static final int M2_ID = 1;
    public static final int M3_ID = 2;
    public static final int M4_ID = 3;    
    
    public static final int POWER_MIN = 0;
    public static final int POWER_MAX = 100;
    
    public Motor[] motors;
   
    public DroneMotors(){
        motors = new Motor[4];
        motors[M1_ID] = new Motor();
        motors[M2_ID] = new Motor();
        motors[M3_ID] = new Motor();
        motors[M4_ID] = new Motor();
    }
    
    public class Motor{
        public Integer power;

        public Motor() {
            this.power = 0;
        }
        
        public Integer getPowerPercent(){
            return (this.power);
        }
        
        public Integer getPowerAngle(){
            return map(this.power, POWER_MIN, POWER_MAX, 0, 360);
        }
        
        private int map(int x, int in_min, int in_max, int out_min, int out_max){
            return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
        }
    }
}
