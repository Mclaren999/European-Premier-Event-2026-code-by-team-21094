package org.firstinspires.ftc.teamcode.subsystems.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@Config
@TeleOp(name = "KP")
public class KPTuner extends OpMode {
    private ShooterSystem shooterSystem;

    public static double kV = 0.002446;

    public static double kS = 0.06525;

    public static double kP = 0.002;

    public static double goal = 0;
    @Override
    public void init(){
        shooterSystem = new ShooterSystem( hardwareMap);
    }

    @Override
    public void loop(){
        double feedForward = (kV * goal)* kS;
        double error = goal - shooterSystem.GetRPM();
        double feedBack = error * kP;
        shooterSystem.setPower(feedBack + feedForward);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("RPM", "%.6f", shooterSystem.GetRPM());
        telemetry.addData("error", "%.6f", error);

    }
}