/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package utils;

import java.awt.Component;
import java.io.FileWriter;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFileChooser;

/**
 *
 * @author Asafe
 */
public class SaveData {
    
    private static final String TAB = "\t";
    private static final String NEW_LINE = "\r\n";
    
    public static void save(Component parent, String[] data){
        JFileChooser file = new JFileChooser();
        int result = file.showSaveDialog(parent);
        
        if(result == JFileChooser.APPROVE_OPTION){
            String nameFile = file.getCurrentDirectory().toString() +
                                "\\" + file.getSelectedFile().getName();
            
            try {
                try (FileWriter writer = new FileWriter(nameFile)) {
                    writer.write(TAB + PidData.KP_NAME +
                            TAB + PidData.KI_NAME +
                            TAB + PidData.KD_NAME + NEW_LINE);
                    writer.write(PidData.YAW_NAME + TAB +
                            data[PidData.YAW_KP] + TAB +
                            data[PidData.YAW_KI] + TAB +
                            data[PidData.YAW_KD] + NEW_LINE);
                    writer.write(PidData.PITCH_NAME + TAB +
                            data[PidData.PITCH_KP] + TAB +
                            data[PidData.PITCH_KI] + TAB +
                            data[PidData.PITCH_KD] + NEW_LINE);
                    writer.write(PidData.ROLL_NAME + TAB +
                            data[PidData.ROLL_KP] + TAB +
                            data[PidData.ROLL_KI] + TAB +
                            data[PidData.ROLL_KD]);
                }
                
            } catch (IOException ex) {
                Logger.getLogger(SaveData.class.getName()).log(Level.SEVERE, null, ex);
            }
            
        }
    }
    
}
