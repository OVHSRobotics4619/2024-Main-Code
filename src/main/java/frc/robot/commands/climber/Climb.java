package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;


import edu.wpi.first.wpilibj.Timer;

public class Climb extends Command {

    private final ClimberSubsystem climberSubsystem;
    private Timer driveTime = new Timer();
    private double Time;

  public Climb(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.climberSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      driveTime.reset();
      driveTime.start();
    }

    @Override
    public void execute() {
      Time = driveTime.get();

      climberSubsystem.climb(Constants.Climber.CLIMB_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climberSubsystem.stopClimb();
    }

  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (Time > Constants.Climber.CLIMB_TIME) {
        return true;
      }
      return false;
    }
};