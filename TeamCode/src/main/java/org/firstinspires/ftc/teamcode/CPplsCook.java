package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CPplsCook", group="Linear OpMode")
@Config
public class CPplsCook extends LinearOpMode {

    // --- Gamepad 1 ---
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake = null;
    private DcMotor intake2 = null;
    private DcMotor shooter = null;
    private Servo shooterHinge = null;
    private CRServo intakeToShooter = null;
    private CRServo intakeToShooter2 = null;




    public static boolean wheelBreak = false;
    // Wheel brake tuning
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;
    public static int wheelBreakTargetFL;
    public static int wheelBreakTargetFR;
    public static int wheelBreakTargetBL;
    public static int wheelBreakTargetBR;

    // --- Gamepad 2 ---

    public static boolean intakeActive = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;
    public static double kP = 0.01;      // Proportional control constant
    public static double maxPower = 0.2; // Maximum motor power (range: 0–1)
    public static int maxError = 100;    // Maximum allowable encoder error

    // Continuous rotation servos

    // Speeds
    public static double intake_speed = 0.1;
    public static double shooter_power = 0.5;
    public static double intakeToShooter_power = 0.5;

    // --- Runtime ---
    private ElapsedTime runtime = new ElapsedTime();

    public static boolean slow_mode = false;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // --- Hardware Mapping ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backl");
        backRightDrive = hardwareMap.get(DcMotor.class, "backr");

        intake = hardwareMap.get(DcMotor.class, "i");
        intake2 = hardwareMap.get(DcMotor.class, "i2");
        shooter = hardwareMap.get(DcMotor.class, "s");
        shooterHinge = hardwareMap.get(Servo.class, "sH");
        intakeToShooter = hardwareMap.get(CRServo.class, "its");
        intakeToShooter2 = hardwareMap.get(CRServo.class, "its2");

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        double nerf = 0.1;

        telemetry.setMsTransmissionInterval(100);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Started!");
        telemetry.update();

        runtime.reset();

        while (opModeIsActive()) {

            // --- Driving control ---
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * 0.60;

            // Slow mode


            // --- Wheel break ---
            if (gamepad1.left_stick_button && gamepad1.right_stick_button && !wheelBreak) {
                wheelBreak = true;
                sleep(200);
                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
                wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
                wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
                wheelBreakTargetBR = backRightDrive.getCurrentPosition();

//                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                frontLeftDrive.setTargetPosition(0);
                frontRightDrive.setTargetPosition(0);
                backLeftDrive.setTargetPosition(0);
                backRightDrive.setTargetPosition(0);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && wheelBreak){
                wheelBreak = false;
                sleep(200);
            }

            if (wheelBreak) {

                // --- Front Left ---
                int errorFL = wheelBreakTargetFL - frontLeftDrive.getCurrentPosition();
                if (errorFL > maxError) {
                    errorFL = maxError;
                } else if (errorFL < -maxError) {
                    errorFL = -maxError;
                }

                double powerFL = kP * errorFL;
                if (powerFL > maxPower) {
                    powerFL = maxPower;
                } else if (powerFL < -maxPower) {
                    powerFL = -maxPower;
                }
                frontLeftDrive.setPower(powerFL);

                // --- Front Right ---
                int errorFR = wheelBreakTargetFR - frontRightDrive.getCurrentPosition();
                if (errorFR > maxError) {
                    errorFR = maxError;
                } else if (errorFR < -maxError) {
                    errorFR = -maxError;
                }

                double powerFR = kP * errorFR;
                if (powerFR > maxPower) {
                    powerFR = maxPower;
                } else if (powerFR < -maxPower) {
                    powerFR = -maxPower;
                }
                frontRightDrive.setPower(powerFR);

                // --- Back Left ---
                int errorBL = wheelBreakTargetBL - backLeftDrive.getCurrentPosition();
                if (errorBL > maxError) {
                    errorBL = maxError;
                } else if (errorBL < -maxError) {
                    errorBL = -maxError;
                }

                double powerBL = kP * errorBL;
                if (powerBL > maxPower) {
                    powerBL = maxPower;
                } else if (powerBL < -maxPower) {
                    powerBL = -maxPower;
                }
                backLeftDrive.setPower(powerBL);

                // --- Back Right ---
                int errorBR = wheelBreakTargetBR - backRightDrive.getCurrentPosition();
                if (errorBR > maxError) {
                    errorBR = maxError;
                } else if (errorBR < -maxError) {
                    errorBR = -maxError;
                }

                double powerBR = kP * errorBR;
                if (powerBR > maxPower) {
                    powerBR = maxPower;
                } else if (powerBR < -maxPower) {
                    powerBR = -maxPower;
                }
                backRightDrive.setPower(powerBR);

                telemetry.addData("WHEEL BRAKE ACTIVE", "True");

            } else if (!wheelBreak){
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                backLeftDrive.setPower(Logdrive + LATdrive - Turndrive);
                backRightDrive.setPower(Logdrive - LATdrive + Turndrive);
                frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);adb disconnect
                frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);
            }

            // --- Intake ---
            if (gamepad2.left_bumper && intakeActive) {
                sleep(200);
                intake.setPower(intake_speed);
                intake2.setPower(intake_speed);
            } else if (gamepad2.left_bumper && !intakeActive) {
                sleep(200);
                intake.setPower(0);
                intake2.setPower(0);
            }



//            if (gamepad2.left_bumper) intakeActive = !intakeActive;
//            sleep(200);
//            intake.setPower(intakeActive ? intake_speed : 0);
//            intake2.setPower(intakeActive ? intake_speed : 0);

            // --- Shooter ---
            if (gamepad2.right_bumper) {
                sleep(200);
                shooterActive = !shooterActive;
                shooter.setPower(shooterActive ? shooter_power : 0);
                intakeToShooter.setPower(shooterActive ? intakeToShooter_power : 0);
                intakeToShooter2.setPower(shooterActive ? intakeToShooter_power : 0);
            }

            // --- Shooter hinge ---
            if (gamepad2.a) {
                sleep(200);
                shooterUp = !shooterUp;
                shooterHinge.setPosition(shooterUp ? 1 : 0);
            }

            if (gamepad1.right_bumper && !slow_mode) {
                sleep(200);
                nerf = 0.1;
                slow_mode = true;
            }
            else if(gamepad1.right_bumper && slow_mode) {
                sleep(200);
                nerf = 0.5;
                slow_mode = false;
            }

            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Wheel Break Active", wheelBreak);
            packet.put("Intake Active", intakeActive);
            packet.put("Shooter Active", shooterActive);
            packet.put("Shooter Hinge Position", shooterHinge.getPosition());
            packet.put("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            packet.put("Front Right Encoder", frontRightDrive.getCurrentPosition());
            packet.put("Back Left Encoder", backLeftDrive.getCurrentPosition());
            packet.put("Back Right Encoder", backRightDrive.getCurrentPosition());
            packet.put("Nerf Speed", nerf);
            packet.put("Slow Mode", slow_mode);


            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(20);

            telemetry.addData("Wheel Break Active", wheelBreak);
            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.addData("Shooter Hinge Position", shooterHinge.getPosition());
            telemetry.addData("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            telemetry.addData("Front Right Encoder", frontRightDrive.getCurrentPosition());
            telemetry.addData("Back Left Encoder", backLeftDrive.getCurrentPosition());
            telemetry.addData("Back Right Encoder", backRightDrive.getCurrentPosition());
            telemetry.addData("Nerf Speed", nerf);
            telemetry.addData("Slow Mode",  slow_mode);


        }
    }
}
