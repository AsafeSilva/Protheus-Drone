/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package Chart;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JPanel;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataItem;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

/**
 *  Helps create and use graphics using the jFreeChart library (http://www.jfree.org/jfreechart/)
 * 
 * @author Asafe Silva
 * @version 1.0
 */
public class Chart{
    
    /**
     * The default range for the charts in x axis.
     */
    public static final int DEFAULT_RANGE = 50;
    
    /**
     * A {@link JFreeChart}.
     */
    private JFreeChart thisChart;
    
    /**
     * Names of the chart title, x axis and y axis.
     */
    private final String chartTitle, xAxisLabel, yAxisLabel;
    
    /**
     * A {@link Map} to store the {@link XYSeries} where the key is the name of the XYSeries.
     */
    private final Map<String, XYSeries> chartSeries;
    
    /**
     * A {@link JPanel} for the {@link JFreeChart} to be displayed.
     */
    private final JPanel jPanel;
    
    /**
     * A listener to be called when a new data is added.
     */
    private ChartDataListener dataListener;
    
    /**
     * A long to store value of 'x' axis
     */
    private long xAxisValue =0;
    
    
    /**
     * Creates a new {@link Chart} with a panel and a title.
     * 
     * @param panel A {@link JPanel} to display the chart. ['null' to not visible]
     * @param title The chart name.
     */
    public Chart(JPanel panel, String title){
        
        this.jPanel = panel;        
        this.chartTitle = title;
        this.xAxisLabel = null;
        this.yAxisLabel = null;
        
        chartSeries = new HashMap<>();
    }
    
    /**
     * Creates a new {@link Chart} with a panel, a title and the axis name [x, y].
     * 
     * @param panel A {@link JPanel} to display the chart. ['null' to not visible]
     * @param title The chart name.
     * @param xAxisLabel The x axis name.    
     * @param yAxisLabel The y axis name.
     */
    public Chart(JPanel panel, String title, String xAxisLabel, String yAxisLabel){
        this.jPanel = panel;
        this.chartTitle = title;
        this.xAxisLabel = xAxisLabel;
        this.yAxisLabel = yAxisLabel;
        
        chartSeries = new HashMap<>();
    }
    
    /**
     * Adds a new {@link XYSeries} in the XYSeries list.
     * 
     * @param name The XYSeries name.
     */
    public void addChart(String name){
        chartSeries.put(name, new XYSeries(name)); 
    }
    
    /**
     * Adds new {@link XYSeries}[s] in the XYSeries list.
     * 
     * @param names A {@link List} of the XYSeries names.
     */
    public void addChart(List<String> names){
        for(String name : names)
            chartSeries.put(name, new XYSeries(name));  
    }

    /**
     * Initialize the chart with yours XYSeries.
     */
    public void initialize(){
        XYSeriesCollection dataset = new XYSeriesCollection();
        
        for(XYSeries s : chartSeries.values()){
            s.setMaximumItemCount(3000);
            dataset.addSeries(s);
        }
        
        thisChart = ChartFactory.createXYLineChart(chartTitle, xAxisLabel, yAxisLabel, dataset);
        
        if(jPanel == null) return;
        
        ChartPanel chartPanel = new ChartPanel(thisChart);
        chartPanel.setSize(jPanel.getWidth(), jPanel.getHeight());
        jPanel.add(chartPanel);
        
        XYPlot plot = thisChart.getXYPlot();
        plot.getDomainAxis().setFixedAutoRange(DEFAULT_RANGE);
        plot.getDomainAxis().setTickLabelsVisible(false);
    }
    
    /**
     * Get the x Axis value.
     * @return The x Axis value.
     */
    public long getXAxisValue() {
        return xAxisValue;
    }
    
    /**
     * Get the {@link XYSeries} with the specified name.
     * 
     * @param name The name of the XYSeries.
     * @return The XYSeries.
     */
    public XYSeries getXYSerie(String name){
        return chartSeries.get(name);
    }
    
    /**
     * Get all {@link XYSeries} of this Chart.
     * 
     * @return A {@link Map} with all the XYSeries and the name as key.
     */
    public Map<String, XYSeries> getListXYSerie(){
        return chartSeries;
    }
    
    /**
     * Adds new value to chart.
     * 
     * @param name The name of the {@link XYSeries}.
     * @param dataItem A new {@link XYDataItem}.
     */
    public void addData(String name, XYDataItem dataItem){
        try{
            chartSeries.get(name).add(dataItem);
            xAxisValue++;
        }catch(NullPointerException ex){
            Logger.getLogger(Chart.class.getName()).log(Level.SEVERE, null, ex);    
        }
        
        if(dataListener != null)
            dataListener.onChartDataListener(name, dataItem);
    }

    /**
     * Clear the {@link XYSeries} with the specified name.
     * 
     * @param name The XYSeries name.
     */
    public void clear(String name){
        chartSeries.get(name).clear();
        xAxisValue = 0;
    }
    
    /**
     * Clear all the {@link XYSeries} of the Chart.
     */
    public void clearAll(){
        for(XYSeries s : chartSeries.values())
            s.clear();
        
        xAxisValue = 0;
    }
    
    /**
     * Set a range for the charts in x axis.
     * 
     * @param range Value of range in x axis.
     */
    public void setFixedAutoRange(int range){
        XYPlot plot = thisChart.getXYPlot();
        plot.getDomainAxis().setFixedAutoRange(range);
    }
    
    /**
     * Sets the maximum number of items that will be retained in the series.
     * 
     * @param maximum The maximum number of items for the series.
     */
    public void setMaximumItemCount(int maximum){
        for(XYSeries s : chartSeries.values()){
            s.setMaximumItemCount(maximum);
        }
    }
    
    /**
     * Sets a new listener to be called when a new data is added.
     * 
     * @param dataListener A new {@link ChartDataListener}
     */
    public void setChartDataListener(ChartDataListener dataListener){
        this.dataListener = dataListener;
    }
    
    /**
     * Listener used when a new data is added.
     */
    public interface ChartDataListener {
        
        /**
         * Called when a new data is added.
         * @param name The name of the XYSeries.
         * @param dataItem A {@link XYDataItem} with the data added.
         */
        public void onChartDataListener(String name, XYDataItem  dataItem);
    }

    /**
     * Override method.
     * 
     * @return Return the title of the chart.
     */
    @Override
    public String toString(){
        return chartTitle;
    }

}
