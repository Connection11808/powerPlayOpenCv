package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ConnectionHardware;

@TeleOp(name="Pushbot: Teleop POV_Connection", group="Pushbot")

public class ConnectionTeleopPOV extends LinearOpMode {

    /* Declare OpMode members. */
    ConnectionHardware robot = new ConnectionHardware();   // Use a Pushbot's hardware
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    public String TAG = "teleop";
    double maxSpeed = 0.6;
    private Elevator elevator = new Elevator();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean armMotorStart = false;
        boolean collectionMotorStart = false;
        int armPosition;
        boolean armMotor_run_to_position = false;




        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.ElivatorMotorUsingEncoder();

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Say", "Hello Driver");    //
        Log.d(TAG, "Say Hello Driver");
        //telemetry.update();
        Log.d(TAG, "update");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        elevator.start();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());

            if (gamepad1.right_bumper) {
                robot.sideDrive(-1.0);
            } else if (gamepad1.left_bumper) {
                robot.sideDrive(1.0);
            } else {

                // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
                // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
                // This way it's also easy to just drive straight, or just turn.
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;

                // Combine drive and turn for blended motion.
                left = drive + turn;
                right = drive - turn;

                // Normalize the values so neither exceed +/- 1.0
                max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0) {
                    left /= max;
                    right /= max;
                }

                if (Math.abs(left) > maxSpeed) {
                    if (left > 0) {
                        left = maxSpeed;
                    } else {
                        left = -maxSpeed;
                    }
                }

                if (Math.abs(right) > maxSpeed) {
                    if (right > 0) {
                        right = maxSpeed;
                    } else {
                        right = -maxSpeed;
                    }
                }

                // Output the safe vales to the motor drives.
                robot.setLeft_frontDrive(left);
                robot.setRight_frontDrive(right);
                robot.setLeft_backDrive(left);
                robot.setRight_backDrive(right);

            }

            if (gamepad1.x) {
                maxSpeed = 1.0;
            } else if (gamepad1.y) {
                maxSpeed = 0.8;
            }

            if (gamepad2.dpad_down) {
                robot.setCatchTheConeMotor(0.6);
                Log.d(TAG, "The position is " + robot.catchTheCone.getPosition());
            } else if (gamepad2.dpad_up) {
                robot.setCatchTheConeMotor(0.9);
                Log.d(TAG, "The position is " + robot.catchTheCone.getPosition());
            }
        }
    }

    private class Elevator extends Thread {
        Elevator() {this.setName("Elevator");}
        float elivatorSpeed = 0.55f;
        boolean stopFlag = false;
        private ElapsedTime runtime = new ElapsedTime();
        public void run()
        {
            try {
                Log.d(TAG, " Elevator thread is starting");

                while ((opModeIsActive())) {

                    if (gamepad2.y) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(4140);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 6) && (gamepad2.left_bumper == false)) {
                          Log.d(TAG , "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position up " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());
                    } else if (gamepad2.a) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(0);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(-elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 6) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    }
                    else if (gamepad2.x) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(300);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 2) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    }
                    else if (gamepad2.b) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(3000);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 4) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    }
                    else if (gamepad2.right_bumper) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(1800);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 3) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    }
                    else {
                        if (gamepad2.right_trigger > 0.3) {
                            stopFlag = false;
                            robot.setElivatorMotor(elivatorSpeed);
                            Log.d(TAG, "Elevator speed is " + elivatorSpeed);
                        }
                        else if (gamepad2.left_trigger > 0.3) {
                            stopFlag = false;
                            robot.setElivatorMotor(-elivatorSpeed);
                            Log.d(TAG, "Elevator speed is " + elivatorSpeed);

                        }
                        else if (gamepad2.dpad_left)
                        {
                            robot.resetEncoderElivatorMotor();
                            Log.d(TAG, "Elevator position is " + robot.getEncoderPositionElivatorMotor());
                        }
                        else {
                            robot.setElivatorMotor(0);
                            if (stopFlag == false) {
                                Log.d(TAG, "Elevator speed is 0");
                                stopFlag = true;
                            }
                        }
                        robot.setElivatorMotor(-0.1);
                        robot.ElivatorMotorUsingBrake();
                    }
                }
                Log.d(TAG, " Elevator thread is finished");
            }
            catch (Exception e)
            {
                Log.d(TAG, "Elevator thread exception: " + e.getMessage());
            }
        }




        }

    }




