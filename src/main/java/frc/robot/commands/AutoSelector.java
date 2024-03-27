package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;

public class AutoSelector extends Command {
    private boolean end = false;
    private ArrayList<Command> autoList = new ArrayList<Command>();
    private int index;

    public AutoSelector(int index) {
        this.index = index;
    }

    public void initialize() {
        // Adds new possible autos
        autoList.add(new AutoGoCommand(Controls.m_driveTrain));     // 1
        autoList.add(new AutoGoCommandLong(Controls.m_driveTrain)); // 2

        // Checks for array out of bounds
        if (autoList.size() >= index + 1) {
            Controls.selectedAutoCommand = autoList.get(index);
        }
        end = true;
    }

    public boolean isFinished() {
        return end;
    }
}
