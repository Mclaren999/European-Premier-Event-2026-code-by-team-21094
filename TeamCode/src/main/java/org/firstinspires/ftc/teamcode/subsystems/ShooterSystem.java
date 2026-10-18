package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class ShooterSystem {

    private final VoltageSensor batterySensor;
    public DcMotorEx leftShooter;
    public DcMotorEx rightShooter;
    private Servo Stopper;

    public static double TARGET_RPM = 1000;
    public static double kV = 0.00169;
    public static double kS = 0.175;
    public static double kP = 0.002;
    public static double KPD = 0.37;
    public static double rand = 14.25;

    public static final double GRAVITY = 386.09;
    public double Angle = Math.toRadians(44.8);
    private static double Height = 30;
    private static double GoalHeight = 29.5;
    private double CameraHeight = 13.0266299;

    public boolean enabled = false;
    public double Vel;

    public static double Stop = 1.00;
    public static double Shoot = 0.20;

    public double FAR_MULT = 1.02;

    public ShooterSystem(HardwareMap hardwareMap) {
        Stopper = hardwareMap.get(Servo.class, "opener");

        leftShooter = hardwareMap.get(DcMotorEx.class, "flywheel2");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightShooter = hardwareMap.get(DcMotorEx.class, "flywheel1");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        VoltageSensor temp = null;
        for (VoltageSensor v : hardwareMap.getAll(VoltageSensor.class)) {
            temp = v;
            break;
        }
        batterySensor = temp;
    }

    public void setFAR_MULT(double farMult) {
        FAR_MULT = farMult;
    }

    public void setTargetRPM(double rpm) {
        TARGET_RPM = rpm;
    }

    public void toggle() {
        enabled = !enabled;
        if (!enabled) stop();
    }

    public void stop() {
        enabled = false;
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void update(double dt) {
        if (!enabled || dt <= 0) return;
        double feedForward = (kV * TARGET_RPM) * kS;
        double error = TARGET_RPM - GetRPM();
        double feedBack = error * kP;
        double total = feedBack + feedForward;
        leftShooter.setPower(total);
        rightShooter.setPower(total);
    }

    public boolean atSetPoint() {
        return TARGET_RPM - GetRPM() <= 400;
    }

    public void Shoot() {
        Stopper.setPosition(Shoot);
    }

    public void setPower(double power) {
        leftShooter.setPower(power);
        rightShooter.setPower(power);
    }

    public double GetRPM() {
        return leftShooter.getVelocity() * 60 / 28;
    }

    public void Stop() {
        Stopper.setPosition(Stop);
    }

    public double calculateRPM(double vel) {
        return (11.95339 + KPD) * vel - 339.45729;
    }

    public void calculateVel(double dis) {
        dis = Math.max(dis, 61.0);
        double denom = 2 * Math.pow(Math.cos(Angle), 2) * (dis * Math.tan(Angle) - Height);
        if (denom <= 0) return;
        Vel = Math.sqrt((GRAVITY * Math.pow(dis, 2)) / denom);
        if (dis >= 130) Vel *= FAR_MULT;
        setTargetRPM(calculateRPM(Vel));
    }

    public double calcDisLime(double ty) {
        double angleToTarget = rand + ty;
        double heightDiff = GoalHeight - CameraHeight;
        return heightDiff / Math.tan(Math.toRadians(angleToTarget));
    }

    public void updateShooter(double distance) {
        calculateVel(distance);
    }

    public double RPM() {
        return calculateRPM(Vel);
    }
}
