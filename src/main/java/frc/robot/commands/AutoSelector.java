package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.Controls;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSelector extends Command {
    private boolean end = false;
    private ArrayList<Command> autoList = new ArrayList<Command>();
    private final BooleanSupplier switch1;
    private final BooleanSupplier switch2;
    private final BooleanSupplier button1;
    private final BooleanSupplier button2;
    private final BooleanSupplier button3;
    private final BooleanSupplier button4;
    private final BooleanSupplier button5;
    private final BooleanSupplier button6;

    private static enum Type {
        SWITCH,
        BUTTON
    }

    private final Type type;

    public AutoSelector(BooleanSupplier switch1, BooleanSupplier switch2) {
        type = Type.SWITCH;
        this.switch1 = switch1;
        this.switch2 = switch2;
        this.button1 = () -> false;
        this.button2 = () -> false;
        this.button3 = () -> false;
        this.button4 = () -> false;
        this.button5 = () -> false;
        this.button6 = () -> false;
    }

    public AutoSelector(BooleanSupplier button1, BooleanSupplier button2, BooleanSupplier button3,
            BooleanSupplier button4, BooleanSupplier button5, BooleanSupplier button6) {
        type = Type.BUTTON;
        this.switch1 = () -> false;
        this.switch2 = () -> false;
        this.button1 = button1;
        this.button2 = button2;
        this.button3 = button3;
        this.button4 = button4;
        this.button5 = button5;
        this.button6 = button6;
    }

    public void initialize() {
        autoList.add(new AutoGoCommand(Controls.m_driveTrain));
        int index = 0;
        if (type == Type.SWITCH) {
            // S1&S2 -> 0
            // S1&!S2 -> 1
            // !S1&S2 -> 2
            // !S1&!S2 -> 3
            if (switch1.getAsBoolean()) {
                if (switch2.getAsBoolean()) {
                    index = 0;
                } else {
                    index = 1;
                }
            } else {
                if (switch2.getAsBoolean()) {
                    index = 2;
                } else {
                    index = 3;
                }
            }
            if (autoList.size() >= index + 1) {
                Controls.selectedAutoCommand = autoList.get(index);
            }
        } else if (type == Type.BUTTON) {
            if (button1.getAsBoolean()) {
                index = 0;
            } else if (button2.getAsBoolean()) {
                index = 1;

            } else if (button3.getAsBoolean()) {
                index = 2;

            } else if (button4.getAsBoolean()) {
                index = 3;

            } else if (button5.getAsBoolean()) {
                index = 4;

            } else if (button6.getAsBoolean()) {
                index = 5;
            }
            if (autoList.size() >= index + 1) {
                Controls.selectedAutoCommand = autoList.get(index);
            }
        }
    }

    public boolean isFinished() {
        return end;
    }
}
