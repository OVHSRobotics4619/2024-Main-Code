package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class Intake extends Command {

    private final ShooterSubsystem shooterSubsystem;

  public Intake(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      shooterSubsystem.setShoot(-Constants.Shooter.INTAKE_SPEED);
      //shooterSubsystem.setIntake(-Constants.Shooter.INTAKE_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shooterSubsystem.stopAll();
    }

  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
};