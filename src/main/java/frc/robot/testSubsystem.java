package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class testSubsystem implements Subsystem{
    TalonFX talon;
    VelocityVoltage m_velocity;

    CANSparkMax spark;
    RelativeEncoder encoder;
    SparkPIDController pid;
    boolean isTalon = false;

    double kS;
    double kV;

    public testSubsystem(boolean isTalon) {
        this.isTalon = isTalon;
        if (isTalon) {
            talon = new TalonFX(10);
            m_velocity = new VelocityVoltage(0);
            var slot0Configs1 = new Slot0Configs();
            slot0Configs1.kP = 0.1;
            slot0Configs1.kI = 0.0;
            slot0Configs1.kD = 0.0;
            slot0Configs1.kS = 0.1;
            slot0Configs1.kV = 0.0;

            talon.getConfigurator().apply(slot0Configs1, 0.050);

            var fx_cfg = new TalonFXConfiguration();
            fx_cfg.Feedback.FeedbackRemoteSensorID = 10;
            fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

            m_velocity.Slot = 0;
        } else if (!isTalon){
            spark = new CANSparkMax(10, MotorType.kBrushless);
            encoder = spark.getEncoder();
            pid = spark.getPIDController();
            pid.setP(0.1);
            pid.setI(0.0);
            pid.setD(0.0);
            kS = 0.1;
            kV = 0.0;
            pid.setFeedbackDevice(encoder);
        }
    }

    public void setVelocity(double velocityInRPM) {
        if (isTalon) talon.setControl(m_velocity.withVelocity(velocityInRPM));
        else if(!isTalon) 
            pid.setReference(
                velocityInRPM,
                ControlType.kVelocity,
                0, 
                kS * Math.signum(velocityInRPM) + kV * velocityInRPM);
    }

    public void setVolts(double volts) {
        if (isTalon) talon.setVoltage(volts);
        else if(!isTalon) spark.setVoltage(volts);
    }

    public void setVoltage(Measure<Voltage> voltage) {
        double volt = voltage.magnitude();
        if (isTalon) talon.setVoltage(volt);
        else if(!isTalon) spark.setVoltage(volt);
    }

    // Used in autonomousInit() in Robot.java
    public void runSysIDTest() {
        if(isTalon) SignalLogger.start();
        else if(!isTalon) {
            DataLogManager.start();
            DataLogManager.logNetworkTables(true);
            URCL.start();
        }

        var sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
            (voltage) -> this.setVoltage(voltage),
        null, // No log consumer, since data is recorded by URCL
            this
        ));

        // The methods below return Command objects
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

        // For rev logs extract using wpilib's log viewer
        // For talon logs extract using phoenix tuner x
        if(isTalon) SignalLogger.stop();
        else if(!isTalon) DataLogManager.stop();
    }
}