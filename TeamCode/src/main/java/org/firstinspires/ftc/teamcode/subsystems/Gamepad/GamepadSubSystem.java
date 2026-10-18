package org.firstinspires.ftc.teamcode.subsystems.Gamepad;


import com.acmerobotics.dashboard.config.Config;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@Config
public class GamepadSubSystem extends GamepadEx {

    public static double SLEW_RATE = 5;

    private final ButtonUtil YButtonUtil;
    private final ButtonUtil BButtonUtil;

    private double previousLeftX = 0;
    private double previousLeftY = 0;
    private double previousRightX = 0;

    public GamepadSubSystem(Gamepad gamepad) {
        super(gamepad);

        ButtonInterface buttonInterfaceY = () -> getButton(GamepadKeys.Button.Y);
        ButtonInterface buttonInterfaceB = () -> getButton(GamepadKeys.Button.B);

        YButtonUtil = new ButtonUtil(buttonInterfaceY);
        BButtonUtil = new ButtonUtil(buttonInterfaceB);
    }

    private double slew(double period, double currentValue, double previousValue) {
        double slewThreshold = SLEW_RATE * period;
        if (Math.abs(currentValue - previousValue) <= slewThreshold)
            return currentValue;

        double multiplier = Math.signum(currentValue - previousValue);
        return Range.clip(previousValue + multiplier * SLEW_RATE * period, -1, 1);
    }

    public double getLeftY(double period) {
        double currentValue = -super.getLeftY();
        double output = slew(period, currentValue, previousLeftY);

        previousLeftY = output;
        return output;
    }

    public double getLeftX(double period) {
        double currentValue = super.getLeftX();
        double output = slew(period, currentValue, previousLeftX);

        previousLeftX = output;
        return output;
    }

    public double getRightX(double period) {
        double currentValue = -super.getRightX();
        double output = slew(period, currentValue, previousRightX);

        previousRightX = output;
        return output;
    }
    public double angleErrorDeg(double current, double target) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
