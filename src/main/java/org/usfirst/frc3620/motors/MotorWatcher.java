package org.usfirst.frc3620.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3620.Utilities;

import java.util.*;

public class MotorWatcher {
  Tracer tracer;

  static public class MotorWatcherInfo {
    MotorWatcherFetcher fetcher;
    EnumSet<MotorWatcherMetric> metrics;
    String name;

    public EnumSet<MotorWatcherMetric> getMetrics() {
      return metrics;
    }

    public String getName() {
      return name;
    }

    public MotorWatcherFetcher getFetcher() {
      return fetcher;
    }
  }

  public void setTracer(Tracer t) {
    tracer = t;
  }

  public MotorWatcher(String name) {
    this.watcherName = Utilities.removeLeadingAndTrailingSlashes(name);
  }

  String watcherName;

  List<MotorWatcherInfo> collectedInformationList = new ArrayList<>();
  boolean collectedInformationListIsFrozen = false;

  public void addMotor(String name, Object o, EnumSet<MotorWatcherMetric> _metrics) {
    MotorWatcherFetcher f = null;
    if (o instanceof TalonFX) {
      f = new TalonFetcher((TalonFX) o);
    } else if (o instanceof SparkMax) {
      f = new SparkMAXFetcher((SparkMax) o);
    }
    if (f != null) {
      MotorWatcherInfo mwi = new MotorWatcherInfo();
      mwi.metrics = _metrics;
      mwi.fetcher = f;
      mwi.name = watcherName + "/" + Utilities.removeLeadingAndTrailingSlashes(name);
      collectedInformationList.add(mwi);
    }
  }

  public void collect(boolean publish) {
    if (! collectedInformationListIsFrozen) freezeCollectedInformation();
    for (var mwi : collectedInformationList) {
      mwi.fetcher.collect(mwi.metrics);
      if (tracer != null) {
        tracer.addEpoch("fetching motor " + mwi.name);
      }
    }

    if (publish) {
      for (var mwi : collectedInformationList) {
        for (var metric : mwi.metrics) {
          Double value = null;
          value = metric.getValue(mwi.fetcher);
          if (value != null) {
            SmartDashboard.putNumber(mwi.name + "/" + metric.getName(), value);
          }
        }
      }
    }
  }

  public List<MotorWatcherInfo> getCollectedInformation() {
    if (! collectedInformationListIsFrozen) freezeCollectedInformation();
    return collectedInformationList;
  }

  /**
   * turn the list of collected information into an unmodifiable list.
   * do this just once, then we do not have the overhead of creating it 
   * each time we want it.
   */
  void freezeCollectedInformation() {
    collectedInformationList = Collections.unmodifiableList(collectedInformationList);
    collectedInformationListIsFrozen = true;
  }
}
