package org.firstinspires.ftc.teamcode.pedro.RobotCode.Commands.TeleOp.Automated;

import org.firstinspires.ftc.teamcode.RobotCode.Constants;
import org.firstinspires.ftc.teamcode.RobotCode.PIDController;
import org.firstinspires.ftc.teamcode.RobotCode.Subsystems.ShooterSubsystem;

public class SetPollenRPM {
    private ShooterSubsystem shooterSubsystem;
    private PIDController pidController;

    public SetPollenRPM(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        pidController = Constants.PayloadConstants.pollenPID;
    }
    public void operate(double rpm, double dt) {
        pidController.createSetPoint(rpm);
        pidController.setProcessVariable(shooterSubsystem.getPollenShooterRPM());
        shooterSubsystem.setPollenShooterPower(pidController.getOutput(dt));
    }
    public void shutdown() {
        pidController.resetIntegral();
        shooterSubsystem.shutdown();
    }
    public double getError() {
        return pidController.getError();
    }
}
