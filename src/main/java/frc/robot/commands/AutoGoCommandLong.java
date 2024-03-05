package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoGoCommandLong extends Command {
    Timer timer;
    private final DriveSubsystem m_driveTrain;

    public AutoGoCommandLong(DriveSubsystem subsystem) {
        this.m_driveTrain = subsystem;
        timer = new Timer();
    }

    public void initialize() {

        timer.reset();
        timer.start();
    }

    public void execute() {
        m_driveTrain.drive(0.1, 0, 0, true, true);
    }

    public void end(boolean interrupted) {
        m_driveTrain.drive(0, 0, 0, true, true);
    }

    public boolean isFinished() {
        if (timer.hasElapsed(10) == true) {
            return true;
        } else {
            return false;
        }
    }
}
