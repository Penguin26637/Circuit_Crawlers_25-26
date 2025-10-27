package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="CPplsCookExpansive", group="Linear OpMode")
@Config
public class CPplsCookExpansive extends LinearOpMode {

    // --- Gamepad 1 ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    public static boolean wheelBreak = false;
    public static int wheelBreakTargetFL, wheelBreakTargetFR, wheelBreakTargetBL, wheelBreakTargetBR;

    // Wheel brake tuning variables
    public static double wheelBreak_kP = 0.01;
    public static double wheelBreak_maxPower = 0.2;
    public static int wheelBreak_maxError = 100;

    // --- Gamepad 2 ---
    private DcMotor intake, intake2, shooter;
    private Servo shooterHinge;
    private CRServo intakeToShooter, intakeToShooter2;

    public static boolean intakeActive = false;
    public static boolean shooterActive = false;
    public static boolean shooterUp = false;

    public static double intake_speed = 0.1;
    public static double shooter_power = 0.5;
    public static double intakeToShooter_power = 0.5;

    // --- Runtime ---
    private ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

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

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Initialize dashboard ---
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double nerf = 0.1;

        while (opModeIsActive()) {

            // --- Driving control ---
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * 0.60;

            // --- Slow mode ---
            if (gamepad1.left_bumper || gamepad1.right_bumper) nerf = 0.1;

            // --- Wheel break ---
            if (gamepad1.left_bumper && gamepad1.right_bumper && !wheelBreak) {
                wheelBreak = true;
                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
                wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
                wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
                wheelBreakTargetBR = backRightDrive.getCurrentPosition();

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if (wheelBreak) {
                // Front Left
                int errorFL = wheelBreakTargetFL - frontLeftDrive.getCurrentPosition();
                errorFL = Math.max(Math.min(errorFL, wheelBreak_maxError), -wheelBreak_maxError);
                double powerFL = Math.max(Math.min(wheelBreak_kP * errorFL, wheelBreak_maxPower), -wheelBreak_maxPower);
                frontLeftDrive.setPower(powerFL);

                // Front Right
                int errorFR = wheelBreakTargetFR - frontRightDrive.getCurrentPosition();
                errorFR = Math.max(Math.min(errorFR, wheelBreak_maxError), -wheelBreak_maxError);
                double powerFR = Math.max(Math.min(wheelBreak_kP * errorFR, wheelBreak_maxPower), -wheelBreak_maxPower);
                frontRightDrive.setPower(powerFR);

                // Back Left
                int errorBL = wheelBreakTargetBL - backLeftDrive.getCurrentPosition();
                errorBL = Math.max(Math.min(errorBL, wheelBreak_maxError), -wheelBreak_maxError);
                double powerBL = Math.max(Math.min(wheelBreak_kP * errorBL, wheelBreak_maxPower), -wheelBreak_maxPower);
                backLeftDrive.setPower(powerBL);

                // Back Right
                int errorBR = wheelBreakTargetBR - backRightDrive.getCurrentPosition();
                errorBR = Math.max(Math.min(errorBR, wheelBreak_maxError), -wheelBreak_maxError);
                double powerBR = Math.max(Math.min(wheelBreak_kP * errorBR, wheelBreak_maxPower), -wheelBreak_maxPower);
                backRightDrive.setPower(powerBR);

                if (gamepad1.left_bumper && gamepad1.right_bumper) wheelBreak = false;
            } else {
                backLeftDrive.setPower(Logdrive + LATdrive - Turndrive);
                backRightDrive.setPower(Logdrive - LATdrive + Turndrive);
                frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);
                frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // --- Intake ---
            if (gamepad2.left_bumper) intakeActive = !intakeActive;
            intake.setPower(intakeActive ? intake_speed : 0);
            intake2.setPower(intakeActive ? intake_speed : 0);

            // --- Shooter ---
            if (gamepad2.right_bumper) {
                shooterActive = !shooterActive;
                shooter.setPower(shooterActive ? shooter_power : 0);
                intakeToShooter.setPower(shooterActive ? intakeToShooter_power : 0);
                intakeToShooter2.setPower(shooterActive ? intakeToShooter_power : 0);
            }

            // --- Shooter hinge ---
            if (gamepad2.a) {
                shooterUp = !shooterUp;
                shooterHinge.setPosition(shooterUp ? 1 : 0);
            }

            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Wheel Break Active", wheelBreak);
            packet.put("Intake Active", intakeActive);
            packet.put("Shooter Active", shooterActive);
            packet.put("Shooter Hinge", shooterHinge.getPosition());
            packet.put("Front Left Encoder", frontLeftDrive.getCurrentPosition());
            packet.put("Front Right Encoder", frontRightDrive.getCurrentPosition());
            packet.put("Back Left Encoder", backLeftDrive.getCurrentPosition());
            packet.put("Back Right Encoder", backRightDrive.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(20);
        }
    }
}
