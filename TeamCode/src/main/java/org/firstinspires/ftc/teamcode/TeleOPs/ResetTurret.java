package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="resetEncoder",group="1")
public class ResetTurret extends LinearOpMode {

    private DcMotorEx turret = null;

    @Override
    public void runOpMode(){
        turret = hardwareMap.get(DcMotorEx.class,"turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
    }
}
