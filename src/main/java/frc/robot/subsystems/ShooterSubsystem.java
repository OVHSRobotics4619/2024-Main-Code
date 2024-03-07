package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class ShooterSubsystem extends SubsystemBase {
  private VictorSPX topLeft   = new VictorSPX(12);
  private VictorSPX topRight = new VictorSPX(14);


  private VictorSPX bottomRight = new VictorSPX(13);
  private VictorSPX bottomLeft = new VictorSPX(15);

  public ShooterSubsystem() {
    this.topRight.setInverted(true);
    this.bottomRight.setInverted(true);
  }

  public void setShoot(double speed) {
    topLeft.set(VictorSPXControlMode.PercentOutput, speed);
    topRight.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void setIntake(double speed) {
    bottomRight.set(VictorSPXControlMode.PercentOutput, speed);
    bottomLeft.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stopIntake() {
    bottomRight.set(VictorSPXControlMode.PercentOutput, 0);
    bottomLeft.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stopShoot() {
    topRight.set(VictorSPXControlMode.PercentOutput, 0);
    topLeft.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void stopAll() {
    topRight.set(VictorSPXControlMode.PercentOutput, 0);
    topLeft.set(VictorSPXControlMode.PercentOutput, 0);
    bottomRight.set(VictorSPXControlMode.PercentOutput, 0);
    bottomLeft.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void shoot() {
    this.setShoot(Shooter.SHOOTING_SPEED);
    this.setIntake(Shooter.INTAKE_SPEED);
  }

  public void intake() {
    this.setIntake(-Shooter.INTAKE_SPEED);
    this.setShoot(-Shooter.INTAKE_SPEED);
  }

}