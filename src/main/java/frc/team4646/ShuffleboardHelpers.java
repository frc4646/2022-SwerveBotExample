package frc.team4646;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

public class ShuffleboardHelpers {

    /**
     * Create a new tab with horizontal grids sized 2x4. Using a DoubleSupplier for the values lets it automatically update the dashboard.
     * <pre>
     *  var valueMap = new LinkedHashMap<String, DoubleSupplier>();
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     * </pre>
     */
    public static ShuffleboardLayout Create_TabWithGrids(String tabName, String gridName, LinkedHashMap<String, DoubleSupplier> values)
    {
        return Create_TabWithGrids(tabName, gridName, 2, 4, values);
    }
    
    /**
     * Create a new tab with horizontal grids of custom size. Using a DoubleSupplier for the values lets it automatically update the dashboard.
     * <pre>
     *  var valueMap = new LinkedHashMap<String, DoubleSupplier>();
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     * </pre>
     */
    public static ShuffleboardLayout Create_TabWithGrids(String tabName, String gridName, int gridWidth, int gridHeight, LinkedHashMap<String, DoubleSupplier> values)
    {
        // get the tab and the current count
        var tab = Shuffleboard.getTab(tabName);
        var currentLayoutIndex = tab.getComponents().size();

        // add our grid to it
        var layout = tab.getLayout(gridName, BuiltInLayouts.kGrid)
            .withSize(gridWidth, gridHeight)
            .withPosition(currentLayoutIndex * gridWidth, 0)
            .withProperties(Map.of("Number of rows", values.size(), "Number of columns", 1));

        // add each value entry to the grid
        values.forEach((title, value) -> {
            var currentValueIndex = layout.getComponents().size();
            layout.addNumber(title, value).withPosition(0, currentValueIndex);
        });

        return layout;
    }
    
    /**
     * Create a new tab with horizontal grids sized 2x4. Lets you consume the response using NetworkTableEntry
     * <pre>
     *  var valueMap = new LinkedHashMap<String, Double>();
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     * </pre>
     */
    public static LinkedHashMap<String, SimpleWidget> Create_TabWithGrids_Editable(String tabName, String gridName, LinkedHashMap<String, Double> values)
    {
        return Create_TabWithGrids_Editable(tabName, gridName, 2, 4, values);
    }
    
    /**
     * Create a new tab with horizontal grids of custom size. Lets you consume the response using NetworkTableEntry
     * <pre>
     *  var valueMap = new LinkedHashMap<String, Double>();
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     *  valueMap.put("Value 1 Name", () -> value1Variable);
     * </pre>
     */
    public static LinkedHashMap<String, SimpleWidget> Create_TabWithGrids_Editable(String tabName, String gridName, int gridWidth, int gridHeight, LinkedHashMap<String, Double> values)
    {
        // get the tab and the current count
        var tab = Shuffleboard.getTab(tabName);
        var currentLayoutIndex = tab.getComponents().size();

        // add our grid to it
        var layout = tab.getLayout(gridName, BuiltInLayouts.kGrid)
            .withSize(gridWidth, gridHeight)
            .withPosition(currentLayoutIndex * gridWidth, 0)
            .withProperties(Map.of("Number of rows", values.size(), "Number of columns", 1));

        var widgets = new LinkedHashMap<String, SimpleWidget>();

        // add each value entry to the grid
        values.forEach((title, value) -> {
            var currentValueIndex = layout.getComponents().size();
            widgets.put(title, layout.add(title, value).withPosition(0, currentValueIndex));
        });

        return widgets;
    }
}
