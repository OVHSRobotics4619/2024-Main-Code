package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


import edu.wpi.first.wpilibj.Timer;

public class AlignShoot extends Command {

    private Timer driveTime = new Timer();
    private double Time;

  public AlignShoot(ShooterSubsystem shooterSubsystem) {
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Auto shooting");
      
      driveTime.reset();
      driveTime.start();
    }

    @Override
    public void execute() {
      Time = driveTime.get();

    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
    }

  
    // Returns true when the command should end
    @Override
    public boolean isFinished() {
      if (Time > (Constants.Shooter.SHOOTING_SHOOT_TIME + Constants.Shooter.SHOOTING_RAMPUP_TIME)) {
        return true;
      }
      return false;
    }
};