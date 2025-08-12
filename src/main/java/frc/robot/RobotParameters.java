package frc.robot;

import org.usfirst.frc3620.RobotParametersBase;

import com.fasterxml.jackson.annotation.JsonProperty;

public class RobotParameters extends RobotParametersBase {
    String swerveDirectoryName;

    double intakeFrontEncoderOffset = 0;
    double intakeRearEncoderOffset = 0;
    double climberEncoderOffset = 0;
    Double driveStatorCurrentLimit = null;
    @JsonProperty("shortLEDStrip") boolean shortLEDStrip = false;

    public String getSwerveDirectoryName() {
        return swerveDirectoryName;
    }

    public double getIntakeFrontEncoderOffset() {
        return intakeFrontEncoderOffset;
    }

    public double getIntakeRearEncoderOffset() {
        return intakeRearEncoderOffset;
    }

    public double getClimberEncoderOffset() {
        return climberEncoderOffset; 
    }

    public Double getDriveStatorCurrentLimit() {
        return driveStatorCurrentLimit;
    }

    public boolean isShortLEDStrip() {
        return shortLEDStrip;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveDirectoryName=" + swerveDirectoryName);
        sb.append (", intakeFrontEncoderOffset=" + intakeFrontEncoderOffset);
        sb.append (", intakeRearEncoderOffset=" + intakeRearEncoderOffset);
        sb.append (", climberEncoderOffset=" + climberEncoderOffset);
        sb.append (", driveStatorCurrentLimit=" + driveStatorCurrentLimit);
        sb.append ("]");
        return sb.toString();
    }


}