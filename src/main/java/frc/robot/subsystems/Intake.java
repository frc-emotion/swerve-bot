package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.SmartMotorControllerOptions;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import static edu.wpi.first.units.Units.Seconds;
//diameter ball wheel 2 inch diamter 
//diamter conveyer bell 1.125

//gear reduction 1:1 wheels 
public class Intake extends SubsystemBase {
    //Config for Hopper
    SmartMotorControllerConfig intakeMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("Intake", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(1))
    .withMotorInverted(false)
    .withIdleMode(MotorMode.BRAKE)
    .withOpenLoopRampRate(Seconds.of(0.25));

    
    SmartMotorControllerConfig rollerMotorConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("roller", TelemetryVerbosity.HIGH)
    .withGearing(new MechanismGearing(1))
    .withMotorInverted(false)
    .withIdleMode(null)
    .withOpenLoopRampRate(Seconds.of(0.5));



    



}
