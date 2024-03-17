package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends Command {

    private final ShooterSubsystem shooterSubsystem;
    private Timer driveTime = new Timer();
    private double Time;

    private double ShootSpeed;

  public Shoot(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooterSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      System.out.println("Shooting");
      
      driveTime.reset();
      driveTime.start();
    }

    @Override
    public void execute() {
      Time = driveTime.get();

      if (Time < Constants.Shooter.SHOOTING_RAMPUP_TIME) {
        shooterSubsystem.setShoot(Constants.Shooter.SHOOTING_SPEED);
      } else {
        shooterSubsystem.setIntake(Constants.Shooter.OUTTAKE_SPEED);
      }
    }

    // Called once the command ends or is interrupted
    @Override
    public void end(boolean interrupted) {
      shooterSubsystem.stopAll();
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