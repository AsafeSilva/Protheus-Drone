/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package utils;

import java.awt.Component;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFileChooser;

/**
 *
 * @author Asafe
 */
public class LoadData {
    
    private static final String TAB = "\t";
 
    public static Float[] load(Component parent){
        
        Float data[] = new Float[PidData.DATA_SIZE];
        
        JFileChooser file = new JFileChooser();
        int result = file.showOpenDialog(parent);
        
        if(result == JFileChooser.APPROVE_OPTION){
            
            String nameFile = file.getCurrentDirectory().toString() + "\\" + file.getSelectedFile().getName();
            
            try {
                FileReader reader = new FileReader(nameFile);
                BufferedReader bufferedReader = new BufferedReader(reader);
                
                String[] line;
                
                bufferedReader.readLine();
                line = bufferedReader.readLine().split(TAB);
                data[PidData.YAW_KP] = Float.valueOf(line[1]);
                data[PidData.YAW_KI] = Float.valueOf(line[2]);
                data[PidData.YAW_KD] = Float.valueOf(line[3]);
                line = bufferedReader.readLine().split(TAB);
                data[PidData.PITCH_KP] = Float.valueOf(line[1]);
                data[PidData.PITCH_KI] = Float.valueOf(line[2]);
                data[PidData.PITCH_KD] = Float.valueOf(line[3]);
                line = bufferedReader.readLine().split(TAB);
                data[PidData.ROLL_KP] = Float.valueOf(line[1]);
                data[PidData.ROLL_KI] = Float.valueOf(line[2]);
                data[PidData.ROLL_KD] = Float.valueOf(line[3]);              
                
                reader.close();
                
            } catch (FileNotFoundException ex) {
                Logger.getLogger(LoadData.class.getName()).log(Level.SEVERE, null, ex);
            } catch (IOException ex) {
                Logger.getLogger(LoadData.class.getName()).log(Level.SEVERE, null, ex);
            }  
        }
        
        return data;
    }
    
}
