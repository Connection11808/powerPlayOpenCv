package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ConnectionHardware;

@TeleOp(name= "Tread", group="Pushbot")

public class ConnectionTeleopPOVTread extends LinearOpMode {

    /* Declare OpMode members. */
    ConnectionHardware robot = new ConnectionHardware();   // Use a Pushbot's hardware
    private drive drive = new drive();
    double clawOffset = 0;                       // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    public String TAG = "teleop";
    double maxSpeed = 0.8;
//    private drive drive = new drive();

    public void runOpMode() {
        boolean armMotorStart = false;
        boolean collectionMotorStart = false;
        int armPosition;
        boolean armMotor_run_to_position = false;
        /*ConnectionTeleopPOVTread.drive.start();




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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

        }
    }

    private class drive extends Thread {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        drive() {
            this.setName("drive");
        }

        @Override
        public void run() {
            try {
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
            } catch (Exception e) {

            }
        }
    }

    private class speedDrive extends Thread {

            speedDrive() {
                this.setName("speedDrive");
            }

            public void run() {
                try {

                    if (gamepad1.x) {
                        maxSpeed = 1.0;
                    } else if (gamepad1.y) {
                        maxSpeed = 0.8;
                    }
                }
                catch (Exception e) {

                }
            }
        }

    private class catchTheCone extends Thread {

        catchTheCone() { this.setName("catchTheCone"); }

        public void run() {
            try {

                if (gamepad2.dpad_down)
                {
                    robot.setCatchTheConeMotor(0.6);
                }
                else if (gamepad2.dpad_up)
                {
                    robot.setCatchTheConeMotor(0.75);
                }
            }
            catch (Exception e) {

            }
        }
    }

    private class elevator extends Thread {
        float elivatorSpeed = 0.55f;
        elevator() { this.setName("elevator"); }

        public void run() {
            try {

                if (gamepad2.y)
                {
                    robot.ElivatorMotorSetTargetPosition(3900);
                    robot.ElivatorMotorToPosition();
                    robot.setElivatorMotor(elivatorSpeed);
                    while (robot.ElivatorMotorIsBusy()) {
                        Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                    }
                    robot.setElivatorMotor(0);
                    Log.d(TAG, "Encoder position up " + robot.getEncoderPositionElivatorMotor());
                }
                else if (gamepad2.a)
                {
                    robot.ElivatorMotorSetTargetPosition(0);
                    robot.ElivatorMotorToPosition();
                    robot.setElivatorMotor(-elivatorSpeed);
                    while (robot.ElivatorMotorIsBusy()) {
                        Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                    }
                    robot.setElivatorMotor(0);
                    Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());

                }
                else {
                    robot.setElivatorMotor(-0.1);
                    robot.ElivatorMotorUsingBrake();
                }

            }
            catch (Exception e) {

            }
        }
    }




}



