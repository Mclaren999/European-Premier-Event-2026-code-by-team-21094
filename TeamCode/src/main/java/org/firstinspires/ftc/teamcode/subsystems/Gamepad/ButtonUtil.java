package org.firstinspires.ftc.teamcode.subsystems.Gamepad;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ButtonUtil {

    private final ButtonInterface buttonInterface;

    private final ElapsedTime pressTimer;
    private final ElapsedTime releaseTimer;

    public ButtonUtil(ButtonInterface buttonInterface) {
        this.buttonInterface = buttonInterface;

        pressTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        releaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public enum ButtonEvent {NONE, SINGLE_CLICK, DOUBLE_CLICK, HOLDING}
}
