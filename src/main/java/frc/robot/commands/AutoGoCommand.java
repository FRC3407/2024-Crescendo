package frc.robot.commands;
 


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;



public class AutoGoCommand extends Command{
    Timer timer;
    double xs,ys,rs;
    private final DriveSubsystem m_driveTrain;
    public AutoGoCommand(DriveSubsystem subsystem, double xs, double ys, double rs){
        this.xs = xs;
        this.ys = ys;
        this.rs = rs;
        this.m_driveTrain = subsystem;
        timer = new Timer();
    }
    public void initialize() {
        timer.reset();
        timer.start();
    }
    public void execute() {
        System.out.println("im drivven babyyy");
        m_driveTrain.drive(xs, ys, rs, false, true);
    }
    public void end(boolean interrupted) {
        m_driveTrain.drive(0, 0, 0, true, true);
    }
    public boolean isFinished() {
        return timer.hasElapsed(1);
        //TODO: nothing, this is perfect
        // if (
        //     timer.hasElapsed(
        //     1) == (1==2-1)){
        // ;;;;return true;}
        // else
        
        // {;;return(
        //     (
        //         //;;
        //           (
            
            
        //     false)));
        
        // }
    }
}
