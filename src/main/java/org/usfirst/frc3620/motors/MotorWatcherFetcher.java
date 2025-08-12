package org.usfirst.frc3620.motors;

import java.util.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

/**
 * if you add fields to this, make sure you add to MotorWatcherMetric!
 */
abstract public class MotorWatcherFetcher {
  Double temperature;
  Double position;
  Double outputCurrent;
  Double velocity;

  public static MotorWatcherFetcher create(Object motor) {
    if (motor == null) throw new NullPointerException();

    if (motor instanceof TalonFX) {
      return new TalonFetcher((TalonFX) motor);
    } else if (motor instanceof SparkMax) {
      return new SparkMAXFetcher((SparkMax) motor);
    }

    throw new IllegalArgumentException("need a TalonFx or SparkMax, got a " + motor.getClass());
  }

  abstract public void setPower(double power);

  /**
   * this should measure, save, and return the motor temperature.
   * @return motor temp (Celsius)
   */
  abstract Double measureTemperature();

  /**
   * this returns the last measured temperature
   * @return last measured temperature
   */
  final public Double getTemperature() {
    return temperature;
  };

  /**
   * measure, save, and return the motor position.
   * @return measured position
   */
  abstract Double measurePosition();

  /**
   * this returns the last measured position.
   * @return last measured position
   */
  final public Double getPosition() {
    return position;
  }

  /**
   * measure, save, and return the output current.
   * @return measured output current
   */
  abstract Double measureOutputCurrent();

  /**
   * return the last measured output current.
   * @return last measured output current
   */
  final public Double getOutputCurrent() {
    return outputCurrent;
  };

  abstract Double measureVelocity();
  
  final public Double getVelocity() {
    return velocity;
  }

  /**
   * override this if needed. this would be used in the case where some work
   * needs to be done once per cycle to do measurements. 
   */
  void startMeasurements() {
  }

  /**
   * override this if needed
   */
  void finalizeMeasurements() {
  }

  public void collect(EnumSet<MotorWatcherMetric> metrics) {
    startMeasurements();
    for (var metric : metrics) {
      metric.measure(this);
    }
    finalizeMeasurements();
  }
}
