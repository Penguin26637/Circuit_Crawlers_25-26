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

    // --- Odometry ---
    private DcMotor odoLeft, odoRight, odoBack;
    public static double TICKS_PER_INCH = 100; // adjust for your encoders
    public static double TRACK_WIDTH = 12;     // distance between left/right odos in inches
    public static double BACK_WHEEL_OFFSET = 7; // lateral distance of back wheel

    double xPos = 0, yPos = 0, heading = 0;
    int prevLeft = 0, prevRight = 0, prevBack = 0;

    // --- Runtime ---
    private ElapsedTime runtime = new ElapsedTime();

    // --- Dashboard ---
    private FtcDashboard dashboard;

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

        odoLeft = hardwareMap.get(DcMotor.class, "odoLeft");   // port 0
        odoRight = hardwareMap.get(DcMotor.class, "odoRight"); // port 1
        odoBack = hardwareMap.get(DcMotor.class, "odoBack");   // port 2

        // --- Motor directions ---
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Zero power behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        prevLeft = odoLeft.getCurrentPosition();
        prevRight = odoRight.getCurrentPosition();
        prevBack = odoBack.getCurrentPosition();

        double nerf = 0.1;

        telemetry.setMsTransmissionInterval(100);
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // --- Driving control ---
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * 0.6;

            // --- Wheel break toggle ---
            if (gamepad1.left_stick_button && gamepad1.right_stick_button && !wheelBreak) {
                wheelBreak = true;
                sleep(200);

                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftDrive.setTargetPosition(0);
                frontRightDrive.setTargetPosition(0);
                backLeftDrive.setTargetPosition(0);
                backRightDrive.setTargetPosition(0);

                frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                wheelBreakTargetFL = frontLeftDrive.getCurrentPosition();
                wheelBreakTargetFR = frontRightDrive.getCurrentPosition();
                wheelBreakTargetBL = backLeftDrive.getCurrentPosition();
                wheelBreakTargetBR = backRightDrive.getCurrentPosition();

            } else if (gamepad1.left_stick_button && gamepad1.right_stick_button && wheelBreak) {
                wheelBreak = false;
                sleep(200);
            }

            if (wheelBreak) {
                // Front Left
                int errorFL = wheelBreakTargetFL - frontLeftDrive.getCurrentPosition();
                if (errorFL > wheelBreak_maxError) errorFL = wheelBreak_maxError;
                else if (errorFL < -wheelBreak_maxError) errorFL = -wheelBreak_maxError;
                double powerFL = wheelBreak_kP * errorFL;
                if (powerFL > wheelBreak_maxPower) powerFL = wheelBreak_maxPower;
                else if (powerFL < -wheelBreak_maxPower) powerFL = -wheelBreak_maxPower;
                frontLeftDrive.setPower(powerFL);

                // Front Right
                int errorFR = wheelBreakTargetFR - frontRightDrive.getCurrentPosition();
                if (errorFR > wheelBreak_maxError) errorFR = wheelBreak_maxError;
                else if (errorFR < -wheelBreak_maxError) errorFR = -wheelBreak_maxError;
                double powerFR = wheelBreak_kP * errorFR;
                if (powerFR > wheelBreak_maxPower) powerFR = wheelBreak_maxPower;
                else if (powerFR < -wheelBreak_maxPower) powerFR = -wheelBreak_maxPower;
                frontRightDrive.setPower(powerFR);

                // Back Left
                int errorBL = wheelBreakTargetBL - backLeftDrive.getCurrentPosition();
                if (errorBL > wheelBreak_maxError) errorBL = wheelBreak_maxError;
                else if (errorBL < -wheelBreak_maxError) errorBL = -wheelBreak_maxError;
                double powerBL = wheelBreak_kP * errorBL;
                if (powerBL > wheelBreak_maxPower) powerBL = wheelBreak_maxPower;
                else if (powerBL < -wheelBreak_maxPower) powerBL = -wheelBreak_maxPower;
                backLeftDrive.setPower(powerBL);

                // Back Right
                int errorBR = wheelBreakTargetBR - backRightDrive.getCurrentPosition();
                if (errorBR > wheelBreak_maxError) errorBR = wheelBreak_maxError;
                else if (errorBR < -wheelBreak_maxError) errorBR = -wheelBreak_maxError;
                double powerBR = wheelBreak_kP * errorBR;
                if (powerBR > wheelBreak_maxPower) powerBR = wheelBreak_maxPower;
                else if (powerBR < -wheelBreak_maxPower) powerBR = -wheelBreak_maxPower;
                backRightDrive.setPower(powerBR);

            } else {
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
                frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);
                frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);
            }

            // --- Intake toggle ---
            if (gamepad2.left_bumper && !intakeActive) {
                sleep(200);
                intake.setPower(intake_speed);
                intake2.setPower(intake_speed);
                intakeActive = true;
            } else if (gamepad2.left_bumper && intakeActive) {
                sleep(200);
                intake.setPower(0);
                intake2.setPower(0);
                intakeActive = false;
            }

            // --- Shooter toggle ---
            if (gamepad2.right_bumper) {
                sleep(200);
                shooterActive = !shooterActive;
                shooter.setPower(shooterActive ? shooter_power : 0);
                intakeToShooter.setPower(shooterActive ? intakeToShooter_power : 0);
                intakeToShooter2.setPower(shooterActive ? intakeToShooter_power : 0);
            }

            // --- Shooter hinge toggle ---
            if (gamepad2.a) {
                sleep(200);
                shooterUp = !shooterUp;
                shooterHinge.setPosition(shooterUp ? 1 : 0);
            }

            // --- Slow mode toggle ---
            if (gamepad1.right_bumper && !slow_mode) {
                sleep(200);
                nerf = 0.1;
                slow_mode = true;
            } else if (gamepad1.right_bumper && slow_mode) {
                sleep(200);
                nerf = 0.5;
                slow_mode = false;
            }

            // --- Odometry update ---
            int leftPos = odoLeft.getCurrentPosition();
            int rightPos = odoRight.getCurrentPosition();
            int backPos = odoBack.getCurrentPosition();

            int deltaLeft = leftPos - prevLeft;
            int deltaRight = rightPos - prevRight;
            int deltaBack = backPos - prevBack;

            prevLeft = leftPos;
            prevRight = rightPos;
            prevBack = backPos;

            double dLeft = deltaLeft / TICKS_PER_INCH;
            double dRight = deltaRight / TICKS_PER_INCH;
            double dBack = deltaBack / TICKS_PER_INCH;

            double dHeading = (dRight - dLeft) / TRACK_WIDTH;
            double dForward = (dLeft + dRight) / 2;
            double dSide = dBack - (dHeading * BACK_WHEEL_OFFSET);

            xPos += dForward * Math.cos(heading) - dSide * Math.sin(heading);
            yPos += dForward * Math.sin(heading) + dSide * Math.cos(heading);
            heading += dHeading;

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

            packet.put("Robot X (in)", xPos);
            packet.put("Robot Y (in)", yPos);
            packet.put("Heading (rad)", heading);
            packet.put("Heading (deg)", Math.toDegrees(heading));

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            sleep(20);
        }
    }
}
