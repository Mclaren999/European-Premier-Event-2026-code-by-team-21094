package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TransferSystem {

    private final DcMotorEx transfer;
    private static final double ON = -1;
    private static final double OFF = 0.0;
    public double Shoot = -0.8;
    private static final double reversen = 1;

    public TransferSystem(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "intake");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TurnOn() {
        transfer.setPower(ON);
    }

    public void Shoot() {
        transfer.setPower(Shoot);
    }

    public void TurnOFF() {
        transfer.setPower(OFF);
    }

    public void Reverse() {
        transfer.setPower(reversen);
    }
}
