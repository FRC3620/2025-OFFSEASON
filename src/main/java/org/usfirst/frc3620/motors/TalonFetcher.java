package org.usfirst.frc3620.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFetcher extends MotorWatcherFetcher {
  TalonFX talonFX;

  public TalonFetcher(TalonFX _talonFX) {
    super();
    talonFX = _talonFX;
  }

  @Override
  public void setPower(double power) {
    talonFX.set(power);
  }

  @Override
  Double measureTemperature() {
    return temperature = talonFX.getDeviceTemp().getValue().in(Celsius);
  }

  @Override
  Double measurePosition() {
    return position = talonFX.getPosition().getValue().in(Rotation);
  }

  @Override
  Double measureOutputCurrent() {
    return outputCurrent = talonFX.getStatorCurrent().getValue().in(Amps);
  }

  @Override
  Double measureVelocity() {
    return velocity = talonFX.getVelocity().getValue().in(RPM);
  }
}
