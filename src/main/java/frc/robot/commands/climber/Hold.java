package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PinSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class Hold extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final PinSubsystem pinSubsystem;
    private Timer driveTime = new Timer();
    private double Time;

  public Hold(ClimberSubsystem climberSubsystem, PinSubsystem pinSubsystem) {
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
    }

    @Override
    public void execute() {
      Time = driveTime.get();

      pinSubsystem.disablePin();
      climberSubsystem.climbArmDown();
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
      if (Time > Constants.Climber.HOLD_TIME) {
        return true;
      }
      return false;
    }
};