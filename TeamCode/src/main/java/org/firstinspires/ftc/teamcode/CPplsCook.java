/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="CPplsCook", group="Linear OpMode")
//@Disabled
public class CPplsCook extends LinearOpMode {

    // Gamepad 1
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private boolean wheelBreak = false;

    //    Gamepad 2
    private DcMotor intake = null;
    private DcMotor intake2 = null;
    private DcMotor shooter = null;
    private Servo shooterHinge;
    private boolean intakeActive = false;
    private boolean shooterActive = false;
    private boolean shooterUp = false;

    private CRServo intakeToShooter;
    private CRServo intakeToShooter2;


    private double intake_speed = 0.1;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*

    Gamepad 1
        Driving
            Forward/back = left stick y
            Strafe left/right = left stick x
            Turn in place = right stick x

        Wheel Lock
           both bumbers

        MIGHT HAPPEN
        Shooter positioning(Drive to correct place?)
            Button B
            Trigger hold down until in position
           */
        /*
        Gamepad 2
            Intake
                Left bumper
            Shooter
                Right bumper
            Shooter Hinge
                Left joystick down
                Right joystick up

         */

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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


        // Will need to be changed depends of wheels and motor setup
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double nerf = 0.1;

        telemetry.setMsTransmissionInterval(100);

        // Wait for the game to start (driver presses START)

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();
        telemetry.addLine("Started!");
        telemetry.update();

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // this controls speed
            double Logdrive = -gamepad1.left_stick_y * nerf;
            double LATdrive = -gamepad1.left_stick_x * nerf;
            double Turndrive = -gamepad1.right_stick_x * 0.60;


            // this is for controller dead zones and inaccurate reading
//            if (Math.abs(Logdrive) < 0.1) Logdrive = 0;

            backLeftDrive.setPower(Logdrive + LATdrive - Turndrive);
            backRightDrive.setPower(Logdrive - LATdrive + Turndrive);
            frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);
            frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);

//           slow mode
            if (gamepad1.left_bumper) {
                nerf = 0.1;
            }
            if (gamepad1.right_bumper) {
                nerf = 0.1;
            }

//          Wheel Break
            if (gamepad1.left_bumper && gamepad1.right_bumper && !wheelBreak) {
                wheelBreak = true;

            }

            if (wheelBreak) {
                telemetry.addData("WHEEL BRAKE STOP DRIVING NOW", "wheel break" + wheelBreak);

                backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//                frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                frontLeftDrive.setPower(0);
//                frontRightDrive.setPower(0);
//                backLeftDrive.setPower(0);
//                backRightDrive.setPower(0);

                telemetry.addData("End Game", "Wheel Break: " + wheelBreak);



                if (gamepad1.left_bumper && gamepad1.right_bumper && wheelBreak) {
                    wheelBreak = false;
                    telemetry.addData("Stopped", "wheelBreak" + wheelBreak);

                }
            }

            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            backLeftDrive.setPower(Logdrive + LATdrive - Turndrive);
            backRightDrive.setPower(Logdrive - LATdrive + Turndrive);
            frontLeftDrive.setPower(Logdrive - LATdrive - Turndrive);
            frontRightDrive.setPower(Logdrive + LATdrive + Turndrive);



            //intake and shooter controls
            if (gamepad2.left_bumper && !intakeActive) {
                intakeActive = true;
            } else if (gamepad2.left_bumper && intakeActive) {
                intakeActive = false;
            }

            if (intakeActive) {
                intake.setPower(intake_speed);
                intake2.setPower(intake_speed);
//                intakeToShooter.setPower(0.5);
//                intakeToShooter2.setPower(0.5);
            } else {
                intake.setPower(0);
                intake2.setPower(0);
//                intakeToShooter.setPower(0);
//                intakeToShooter2.setPower(0);
            }


            if (gamepad2.right_bumper && !shooterActive) {
                shooter.setPower(0.5);
                shooterActive = true;
                intakeToShooter.setPower(0.5);
                intakeToShooter2.setPower(0.5);
            } else if (gamepad2.right_bumper && shooterActive) {
                shooter.setPower(0);
                shooterActive = false;
                intakeToShooter.setPower(0);
                intakeToShooter2.setPower(0);
            }


            if (gamepad2.a && !shooterUp) {
                shooterHinge.setPosition(1);
            } else if (gamepad2.a && shooterUp) {
                shooterHinge.setPosition((0));
            }


//             Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor", "Nerf: " + nerf); // motor speed
            telemetry.addData("End Game", "Wheel Break: " + wheelBreak);
            telemetry.addData("Intake", "intake active: " + intakeActive); // intake on / off
            telemetry.addData(" Shooter", "Shooter Active: " + shooterActive); // shooter on / off
            telemetry.addData("Shooter Hinge Position", shooterHinge.getPosition());// shooter hinge position
//            telemetry.log().add("Shooter toggled ON", shooterActive);
            telemetry.addData("status", "running");

            telemetry.update();
            sleep(20);

        }
    }
}
