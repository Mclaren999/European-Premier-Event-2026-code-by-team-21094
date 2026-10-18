package org.firstinspires.ftc.teamcode.subsystems.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@Config
@TeleOp(name = "KV TUNER")
public class KVTuner extends OpMode {
    private ShooterSystem shooterSystem;
    public FtcDashboard dashboardd;

    public static double kV = 0.002446;

    public static double kS = 0.06525;


    public static double goal = 1500;
    @Override
    public void init(){
        shooterSystem = new ShooterSystem(hardwareMap);
        FtcDashboard dashboardd = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboardd.getTelemetry());
    }

    @Override
    public void loop(){
        double power = (kV * goal)* kS;
        shooterSystem.setPower(power);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("RPM", "%.6f", shooterSystem.GetRPM());
        telemetry.addData("G RPM", "%.6f", goal);

    }
}
