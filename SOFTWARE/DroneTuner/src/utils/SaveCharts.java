/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package utils;

import Chart.Chart;
import java.awt.Component;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JFileChooser;
import org.jfree.data.xy.XYDataItem;
import org.jfree.data.xy.XYSeries;

/**
 *
 * @author Asafe
 */
public class SaveCharts {
    
    private static final String TAB = "\t";
    private static final String NEW_LINE = "\r\n";
    
    public static void save(Component parent, List<Chart> charts){
        JFileChooser file = new JFileChooser();
        file.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
        int result = file.showSaveDialog(parent);
        
        if(result != JFileChooser.APPROVE_OPTION) return;
        
        String folderName = file.getCurrentDirectory().toString() +
                                "\\" + file.getSelectedFile().getName();
        
        
        charts.forEach((chart) -> {
            
            String chartTitle = chart.toString();
            Map<String, XYSeries> listXYSerie = chart.getListXYSerie();

            FileWriter writer = null;
            try {
                writer = new FileWriter(folderName + "\\" + chartTitle + ".xls");
                
                for(String name : listXYSerie.keySet())
                    writer.write(name + TAB + TAB);
                
                writer.write(NEW_LINE + NEW_LINE);
        
                List<XYSeries> series = new ArrayList<>(listXYSerie.values());
                int smallerItemCount = 999999;
                for(XYSeries serie : series){
                    if(serie.getItemCount() < smallerItemCount)
                        smallerItemCount = serie.getItemCount();
                }
                
                for (int i = 0; i < smallerItemCount; i++) {
                    for(XYSeries serie : series){
                        List<XYDataItem> items = serie.getItems();
                        
                        String xValue = String.format("%.2f", items.get(i).getX().floatValue()).replace('.', ',');
                        String yValue = String.format("%.2f", items.get(i).getY().floatValue()).replace('.', ',');
                        
                        writer.write(xValue + TAB + yValue + TAB);
                    }
                    
                    writer.write(NEW_LINE);
                }
                
            } catch (IOException ex) {
                Logger.getLogger(SaveCharts.class.getName()).log(Level.SEVERE, null, ex);
            } finally {
                if(writer != null) try {
                    writer.close();
                } catch (IOException ex) {
                    Logger.getLogger(SaveCharts.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
          
        });
    }
    
}
