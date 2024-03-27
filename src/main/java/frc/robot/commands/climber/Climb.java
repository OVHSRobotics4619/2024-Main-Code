package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PinSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class Climb extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final PinSubsystem pinSubsystem;
    private Timer driveTime = new Timer();
    private double Time;

  public Climb(ClimberSubsystem climberSubsystem, PinSubsystem pinSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.pinSubsystem = pinSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem, pinSubsystem);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      driveTime.reset();
      driveTime.start();
      System.out.println("Climber Climbing");
    }

    @Override
    public void execute() {
      Time = driveTime.get();

      climberSubsystem.climbArmDown();
      pinSubsystem.enablePin();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      climberSubsystem.stopClimb();
      pinSubsystem.disablePin();
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