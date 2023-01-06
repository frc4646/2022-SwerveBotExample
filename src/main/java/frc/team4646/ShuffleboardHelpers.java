package frc.team4646;

import java.util.LinkedHashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class ShuffleboardHelpers {

    public static ShuffleboardLayout setRowsColumns(ShuffleboardLayout layout, int rows, int columns) {
        return layout.withProperties(Map.of("Number of rows", rows, "Number of columns", columns));
    }

    /**
     * Create a new tab with horizontal grids sized 2x4. Lets you consume the response using NetworkTableEntry
     * <pre>
     *  var valueMap = new LinkedHashMap<String, Double>();
     *  valueMap.put("Value 1 Name", value1Variable);
     *  newValue1Variable = widgets.get("Value 1 Name").getDouble(defaultValue1);
     * </pre>
     */
    public static LinkedHashMap<String, NetworkTableEntry> Create_TabWithGrids_Editable(String tabName, String gridName, LinkedHashMap<String, Double> values)
    {
        return Create_TabWithGrids_Editable(tabName, gridName, 2, 4, values);
    }
    
    /**
     * Create a new tab with horizontal grids of custom size. Lets you consume the response using NetworkTableEntry
     * <pre>
     *  var valueMap = new LinkedHashMap<String, Double>();
     *  valueMap.put("Value 1 Name", value1Variable);
     *  newValue1Variable = widgets.get("Value 1 Name").getDouble(defaultValue1);
     * </pre>
     */
    public static LinkedHashMap<String, NetworkTableEntry> Create_TabWithGrids_Editable(String tabName, String gridName, int gridWidth, int gridHeight, LinkedHashMap<String, Double> values)
    {
        // get the tab and the current count
        var tab = Shuffleboard.getTab(tabName);
        var currentLayoutIndex = tab.getComponents().size();

        // add our grid to it
        var layout = tab.getLayout(gridName, BuiltInLayouts.kGrid)
            .withSize(gridWidth, gridHeight)
            .withPosition(currentLayoutIndex * gridWidth, 0)
            .withProperties(Map.of("Number of rows", values.size(), "Number of columns", 1));

        var widgets = new LinkedHashMap<String, NetworkTableEntry>();

        // add each value entry to the grid
        var prevCount = layout.getComponents().size();
        int i = 0;
        for(String title : values.keySet()) {
            var value = values.get(title);
            widgets.put(title, layout.add(title, value).withPosition(0, prevCount + i).getEntry());
            i++;
        }

        return widgets;
    }
}
