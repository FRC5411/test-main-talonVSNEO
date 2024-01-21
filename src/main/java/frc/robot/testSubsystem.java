package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    int motorID = 10;

    public testSubsystem(boolean isTalon) {
        this.isTalon = isTalon;
        if (isTalon) {
            talon = new TalonFX(motorID);
            talon.setNeutralMode(NeutralModeValue.Brake);
            m_velocity = new VelocityVoltage(0);

            var fx_cfg = new TalonFXConfiguration();
            fx_cfg.Slot0.kP = 0.1;
            fx_cfg.Slot0.kI = 0.0;
            fx_cfg.Slot0.kD = 0.0;
            fx_cfg.Slot0.kS = 0.0;
            fx_cfg.Slot0.kV = 0.0;
            fx_cfg.Feedback.FeedbackRemoteSensorID = motorID;
            fx_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            fx_cfg.CurrentLimits.StatorCurrentLimit = 40;
            fx_cfg.CurrentLimits.SupplyCurrentLimit = 40;

            m_velocity.Slot = 0;
        } else if (!isTalon){
            spark = new CANSparkMax(motorID, MotorType.kBrushless);
            spark.setIdleMode(IdleMode.kBrake);
            spark.setSmartCurrentLimit(40);
            encoder = spark.getEncoder();
            pid = spark.getPIDController();
            pid.setP(0.1);
            pid.setI(0.0);
            pid.setD(0.0);
            kS = 0.0;
            kV = 0.0;
            pid.setFeedbackDevice(encoder);
        }
    }

    public void setVelocity(double velocityInRPM) {
        SmartDashboard.setDefaultNumber("VelocitySetpoint", velocityInRPM);
        if (isTalon) talon.setControl(m_velocity.withVelocity(velocityInRPM));
        else if(!isTalon) 
            pid.setReference(
                velocityInRPM,
                ControlType.kVelocity,
                0,
                kS * Math.signum(velocityInRPM) + kV * velocityInRPM);
    }

    public void setVoltageWithUnits(Measure<Voltage> voltage) {
        double volt = voltage.magnitude();
        setVolts(volt);
    }

    public void setVolts(double volts) {
        if (isTalon) talon.setVoltage(volts);
        else if(!isTalon) spark.setVoltage(volts);
    }

    // Used in autonomousInit() in Robot.java
    public Command getSysIDTests() {
        SysIdRoutine sysIdRoutine = 
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    Units.Volts.of(1).per(Units.Seconds.of(1)), 
                    Units.Volts.of(4), 
                    Units.Seconds.of(15),
                    (state) -> sysIDStateLogger(state.toString())
                ),
                new SysIdRoutine.Mechanism(
                (voltage) -> this.setVoltageWithUnits(voltage),
            null, // No log consumer, since data is recorded by URCL
                this ) );

        return 
            new SequentialCommandGroup(
                startLoggingRoutine(),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward), Commands.waitSeconds(3),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse), Commands.waitSeconds(3),
                sysIdRoutine.dynamic(    SysIdRoutine.Direction.kForward), Commands.waitSeconds(3),
                sysIdRoutine.dynamic(    SysIdRoutine.Direction.kReverse), Commands.waitSeconds(3),
                stopLoggingRoutine() );
        // For rev logs extract using wpilib's data log tool: https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog-download.html
        // For talon logs extract using phoenix tuner x: https://pro.docs.ctr-electronics.com/en/latest/docs/tuner/tools/log-extractor.html
    }

    public Command startLoggingRoutine() {
        Command startLogging = 
            Commands.runOnce(()-> {
                if(isTalon) SignalLogger.start();
                else if(!isTalon) {
                    DataLogManager.start();
                    DataLogManager.logNetworkTables(true);
                    URCL.start();
                }
            }, this );
        return startLogging;
    }

    public void sysIDStateLogger(String state) {
        if(isTalon) SignalLogger.writeString("Talon/sysIDTestState", state);
        else if(!isTalon) {
                StringLogEntry logEntry = new StringLogEntry(DataLogManager.getLog(), "SystIDState");
                logEntry.append(state, (long)Timer.getFPGATimestamp());
        };
    }

    public Command stopLoggingRoutine() {
        Command stopLogging = 
            Commands.runOnce(()-> { 
                if(isTalon) SignalLogger.stop(); 
                else if(!isTalon) DataLogManager.stop(); 
            }, this);
        return stopLogging;
    }

    @Override
    public void periodic() {
        SmartDashboard.setDefaultNumber("VelocityMeasure", talon.getVelocity().getValueAsDouble());
    }
}