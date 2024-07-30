package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ThrottleShootCommand extends Command{
  private final ShooterSubsystem shooter;
  private final DoubleSupplier throttle;

  public ThrottleShootCommand(
    ShooterSubsystem shooter,
    DoubleSupplier throttle
  ) {

    this.shooter = shooter;
    this.throttle = throttle;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    double throttlePercent = Math.abs(throttle.getAsDouble() - 1) / 2;

    if(throttlePercent < 0.05) {
        throttlePercent = 0;
    }

    shooter.setShooterPct(throttlePercent);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterPct(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}