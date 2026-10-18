package org.firstinspires.ftc.teamcode.subsystems.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSystem;

@Config
@TeleOp(name = "KS")
public class KSTuner extends OpMode {
    private ShooterSystem shooterSystem;

    public static double kS = 0.175;
    @Override
    public void init(){
        shooterSystem = new ShooterSystem(hardwareMap);
    }

    @Override
    public void loop(){
        shooterSystem.setPower(kS);
        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("RPM", "%.6f", shooterSystem.GetRPM());


    }
}
