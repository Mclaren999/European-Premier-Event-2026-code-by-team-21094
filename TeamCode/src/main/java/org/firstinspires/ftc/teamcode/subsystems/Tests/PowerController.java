package org.firstinspires.ftc.teamcode.equipment.controller.shooter;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Configurable
public class PowerController {

    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private VoltageSensor voltageSensor;

    private double directionPos = 0.7;

//        public static double kS = 1.1;    // Volts - static friction
//        public static double kV = 0.0048;  // Volts per RPM - velocity constant
//        public static double kA = 0.001;  // Volts per RPM/s - acceleration constant
//
//        public static double kP = 0.025;
//        public static double kI = 0.0;
//        public static double kD = 0;

    public static double kS = 1.6;    // Volts - static friction
    public static double kV = 0.0033;  // Volts per RPM - velocity constant
    public static double kA = 0.0008;  // Volts per RPM/s - acceleration constant
    private double KofI = 0.001;

    public static double kP = 0.025;
    public static double kI = 0;
    public static double kD = 0.006;


    private double targetVelocityRPM = 0;
    private double lastError = 0;
    private double integral = 0;
    private double lastVelocity = 0;
    private long lastTime = 0;

    private static final double NOMINAL_VOLTAGE = 12.5;

    public static double servoClose = 0.3;
    public static double servoMedium = 0.45;
    public static double servoVeryClose = 0.25;

    private double currentVelocity;

    private enum encoderState {

        rightMotor,
        leftMotor,
        duoMotor,
        nullMotor;

        @Override
        public String toString() {
            switch (this) {
                case rightMotor:
                    return "🛑 right";
                case leftMotor:
                    return "📥 leftMotor";
                case duoMotor:
                    return "🎯 duo";
                default:
                    return "Null";

            }
        }

    }

    public static enum ServosPos {
        DIRECTION_DOWN(0.6),
        DIRECTION_UP(0.3);


        private final double position;

        ServosPos(double pos) {
            this.position = pos;
        }


        public double getPos() {
            return position;
        }
    }

    public void initialize(HardwareMap hardwareMap, String shooterMotorLeftName, String shooterMotorRightName ) {
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, shooterMotorLeftName);
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, shooterMotorRightName);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterMotorLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
//            shooterMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTime = System.nanoTime();
    }

    public boolean checkVelocity(double targetVelocity, double offset) {
        if (Math.abs(shooterMotorLeft.getVelocity()) >= Math.abs(targetVelocity - offset)) {
            return true;
        } else {
            return false;
        }
    }

    public void setShooterVelocity(double targetRPM) {
        this.targetVelocityRPM = targetRPM;
    }

    public void setDirectionPos(double setPos) {
        directionPos = setPos;
    }

    public void update() {

        encoderState encoderState = switchShooterEncoder();

        if (targetVelocityRPM == 0) {

            shooterMotorLeft.setPower(0);
            shooterMotorRight.setPower(0);
            integral = 0;
            lastError = 0;
            return;

        }

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;

        currentVelocity =  getCurrentVelocity(encoderState);

        double acceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;

        double currentVoltage = voltageSensor.getVoltage();
        //        double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

        double feedforwardVoltage =
                kS * Math.signum(targetVelocityRPM) +
                        kV * targetVelocityRPM +
                        kA * acceleration;

        double error = targetVelocityRPM - currentVelocity;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidVoltage = kP * error + KofI * integral + kD * derivative;

        double totalVoltage = feedforwardVoltage + pidVoltage;

        double power = totalVoltage / currentVoltage ;

        //        power *= voltageCompensation;

        power = Math.max(-1.0, Math.min(1.0, power));

        if (encoderState != PowerController.encoderState.nullMotor) {

            shooterMotorLeft.setPower(power);
            shooterMotorRight.setPower(power);

        } else {
            if (Math.abs(targetVelocityRPM) > 0){
                shooterMotorLeft.setPower(0.5);
                shooterMotorRight.setPower(0.5);
            }else {

                shooterMotorLeft.setPower(0);
                shooterMotorRight.setPower(0);

            }
        }
    }

    public boolean isReadyToShoot(double tolerance) {
        if (targetVelocityRPM == 0) return false;
        double currentVelocity = shooterMotorLeft.getVelocity();
        return Math.abs(currentVelocity - targetVelocityRPM) < tolerance;
    }

    public void useKi(boolean isUse){
        if (isUse){
            KofI = kI;
        }else {
            KofI = 0;
        }
    }

    private encoderState switchShooterEncoder(){
        double velShooterRight = shooterMotorRight.getVelocity();
        double velShooterLeft = shooterMotorLeft.getVelocity();

        if (velShooterLeft != 0 && velShooterRight != 0){
            return encoderState.duoMotor;
        } else if (velShooterLeft != 0) {
            return encoderState.leftMotor;
        } else if (velShooterRight != 0) {
            return encoderState.rightMotor;
        }else {
            return encoderState.nullMotor;
        }
    }

    private double getCurrentVelocity(encoderState encoderState){

        switch (encoderState){

            case duoMotor:

                return shooterMotorLeft.getVelocity() + shooterMotorRight.getVelocity() / 2;


            case rightMotor:

                return shooterMotorRight.getVelocity();

            case leftMotor:

                return shooterMotorLeft.getVelocity();

            default:
                return 52;

        }

    }

//        Жанс лолх еюал его в  поппу и ообще он какашка


    public void showTelemetry(Telemetry telemetry) {
        double currentVoltage = voltageSensor.getVoltage();
        double voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;
        encoderState encoderState = switchShooterEncoder();

        telemetry.addLine("=== WPILib Shooter Controller ===");
        telemetry.addData("Left Encoder: ", shooterMotorLeft.getVelocity());
        telemetry.addData("Right Encoder: ", shooterMotorRight.getVelocity());
        telemetry.addData("Target RPM", "%.1f", targetVelocityRPM);
        telemetry.addData("Current RPM", "%.1f", currentVelocity);
        telemetry.addData("Error RPM", "%.1f", targetVelocityRPM - currentVelocity);
        telemetry.addData("Battery Voltage", "%.2fV", currentVoltage);
        telemetry.addData("Voltage Compensation", "%.3f", voltageCompensation);
        telemetry.addData("Ready to Shoot", isReadyToShoot(50) ? "YES" : "NO");
        telemetry.addData("Motor Power", "%.3f", shooterMotorLeft.getPower());
        telemetry.addData("State: ", encoderState)
        ;        }
}
//by chpenya

