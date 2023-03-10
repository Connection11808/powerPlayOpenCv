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
    //private Elevator elevator = new Elevator();

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double armPower = 0.30;
        boolean armMotorStart = false;
        boolean collectionMotorStart = false;
        int armPosition;
        boolean armMotor_run_to_position = false;
        //float elivatorSpeed = 0.85f;
        boolean triggersPress = false;
        boolean elevatorIsMoving = false;
        double positionServo = 0.0;
        double AddGrip = 0.05;
        boolean pres = false;
        int resetPosition = 0;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.ElivatorMotorUsingEncoder();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("Status", "Initialized  Finish");
        Log.d(TAG, "Initialized  Finish");
        telemetry.update();
        //Log.d(TAG, "update");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //elevator.start();
        telemetry.addData("Status", "Start playing...");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());

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

            if (gamepad1.x == true) {
                maxSpeed = 1.0;
            } else if (gamepad1.y == true) {
                maxSpeed = 0.6;
            }

            if (gamepad1.a)
            {
                //robot.resetEncoderArmMotor();
                resetPosition = robot.getEncoderPositionArmMotor();
                Log.d(TAG, "The position of arm_Motor is (reset)" + resetPosition);
                //robot.arm_motor.setTargetPosition(-5);
                //robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //robot.arm_motor.setPower(0.4);
             }

            if (gamepad2.dpad_down == true) {
                robot.setCatchTheConeMotor(0.6);
//                Log.d(TAG, "The position of catchTheCone is " + robot.catchTheCone.getPosition());
                robot.elevatorMotor.setTargetPosition(100);
            } else if (gamepad2.dpad_up == true) {
                robot.setCatchTheConeMotor(0.92);
                Log.d(TAG, "The position of catchTheCone is " + robot.catchTheCone.getPosition());
            }

            if(gamepad2.dpad_right == true){
                triggersPress = false;
                robot.setTuningClawMotor(0.25);
                Log.d(TAG, "The position of arm is (dpad_right is press) " + robot.arm_motor.getCurrentPosition());
                robot.arm_motor.setTargetPosition(resetPosition);
                if(robot.arm_motor.getCurrentPosition() > 0){
                    armPower = armPower * -1;
                }
                robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm_motor.setPower(armPower);

            } else if(gamepad2.dpad_left == true){
                triggersPress = false;
                robot.setTuningClawMotor(0.05);
                Log.d(TAG, "The position of arm is (dpad_left is press) " + robot.arm_motor.getCurrentPosition());
                robot.arm_motor.setTargetPosition(257);
                if(robot.arm_motor.getCurrentPosition() > 257){
                    armPower = armPower * -1;
                }
                robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm_motor.setPower(armPower);


            }else if(gamepad2.right_bumper == true){
                triggersPress = false;
                robot.setTuningClawMotor(0.7);
                robot.arm_motor.setTargetPosition(813);
                if(robot.arm_motor.getCurrentPosition() > 813){
                    armPower = armPower * -1;
                }
                robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm_motor.setPower(0.2);
                robot.setCatchTheConeMotor(0.95);
            }
            else
            {
                if (gamepad2.left_trigger > 0.4)
                {
                    //robot.armMotorWithoutEncoder();
                    //robot.setArmMotor(0.4);
                    //triggersPress = true;
                    int armEncoderPosition = robot.arm_motor.getCurrentPosition();
                    triggersPress = false;
                    armEncoderPosition = armEncoderPosition + 10;
                    robot.arm_motor.setTargetPosition(armEncoderPosition);
                    robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm_motor.setPower(0.4);
                    Log.d(TAG, "The position of arm is (left_trigger is press) " + armEncoderPosition);
                }
                else if (gamepad2.right_trigger > 0.4)
                {
                    //robot.armMotorWithoutEncoder();
                    //robot.setArmMotor(-0.4);
                    //triggersPress = true;
                    int armEncoderPosition = robot.arm_motor.getCurrentPosition();
                    triggersPress = false;
                    armEncoderPosition = armEncoderPosition - 10;
                    robot.arm_motor.setTargetPosition(armEncoderPosition);
                    robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm_motor.setPower(0.4);
                    Log.d(TAG, "The position of arm is (right_trigger is press) " + armEncoderPosition);
                }
                else
                {
                    /*if (triggersPress == true)
                    {
                        int armEncoderPosition = robot.arm_motor.getCurrentPosition();
                        triggersPress = false;
                        armEncoderPosition = armEncoderPosition -10;
                        robot.arm_motor.setTargetPosition(armEncoderPosition);
                        robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.arm_motor.setPower(0.4);
                        Log.d(TAG, "The position of arm is (armEncoderPosition = true) " + armEncoderPosition);
                    }*/
                }
            }

            //Log.d(TAG, "The position of arm is " + robot.arm_motor.getCurrentPosition());


            if (gamepad2.y == true) {
                Log.d(TAG, "press A");
                robot.elevatorMotorWithoutEncoder();
                elevatorIsMoving = true;
                robot.setElivatorMotor(1.0);
                Log.d(TAG, "Elevator speed is " + robot.elevatorMotor.getPower());
                Log.d(TAG, "The position of elevator is " + robot.elevatorMotor.getCurrentPosition());
            } else if (gamepad2.a == true) {
                Log.d(TAG, "press Y");
                robot.elevatorMotorWithoutEncoder();
                elevatorIsMoving = true;
                robot.setElivatorMotor(-1.0);
                Log.d(TAG, "Elevator speed is " + robot.elevatorMotor.getPower());
                Log.d(TAG, "The position of elevator is " + robot.elevatorMotor.getCurrentPosition());

            } else {
                robot.setElivatorMotor(0);
                robot.ElivatorMotorUsingBrake();
                /*if (elevatorIsMoving == true)
                {
                    Log.d(TAG, "press nothing");
                    robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    double elevatorPower = robot.elevatorMotor.getPower();
                    int elevatorPosition = robot.elevatorMotor.getCurrentPosition();
                    Log.d(TAG, "power = " + elevatorPower + " , position = " + elevatorPosition);
                    elevatorIsMoving = false;
                    if (elevatorPower >= 0)
                    {
                        elevatorPosition = elevatorPosition + 20;
                    }
                    else
                    {
                        elevatorPosition = elevatorPosition - 20;
                    }
                    robot.elevatorMotor.setTargetPosition(elevatorPosition);
                    robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.elevatorMotor.setPower(elevatorPower);
                    Log.d(TAG, "The position of elevator is (elevatorIsMoving = true) " + elevatorPosition);
                }*/
        }


        }

    }


    /*private class Elevator extends Thread {
        Elevator() {
            this.setName("Elevator");
        }

        float elivatorSpeed = 0.85f;
        boolean stopFlag = false;
        private ElapsedTime runtime = new ElapsedTime();

        public void run() {
            try {
                Log.d(TAG, " Elevator thread is starting");

                while ((opModeIsActive())) {

                    if (gamepad2.y) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(1958);
                        Log.d(TAG, "ElivatorMotorSetTargetPosition = " + robot.getEncoderPositionElivatorMotor());
                        robot.ElivatorMotorToPosition();
                        Log.d(TAG, "ElivatorMotor is GoToPosition");
                        robot.setElivatorMotor(elivatorSpeed);
                        Log.d(TAG, "ElivatorMotor get power = " + elivatorSpeed);
                        runtime.reset();
                        Log.d(TAG, "runtime reset");
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 6) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
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

                    } else if (gamepad2.x) {
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

                    } else if (gamepad2.b) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(1140);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 4) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    } else if (gamepad2.right_bumper) {
                        stopFlag = false;
                        robot.ElivatorMotorSetTargetPosition(-875);
                        robot.ElivatorMotorToPosition();
                        robot.setElivatorMotor(elivatorSpeed);
                        runtime.reset();
                        while ((robot.ElivatorMotorIsBusy()) && (opModeIsActive()) && (runtime.seconds() < 3) && (gamepad2.left_bumper == false)) {
                            Log.d(TAG, "Encoder position of ElivatorMotor is " + robot.getEncoderPositionElivatorMotor());
                        }
                        robot.setElivatorMotor(0);
                        Log.d(TAG, "Encoder position down " + robot.getEncoderPositionElivatorMotor());
                        Log.d(TAG, "Time elevator is " + runtime.seconds());

                    } else {
                        if (gamepad2.right_stick_y > 0.3) {
                            stopFlag = false;
                            robot.setElivatorMotor(1.0);
                            Log.d(TAG, "Elevator speed is " + elivatorSpeed);
                        } else if (gamepad2.right_stick_y < 0.3) {
                            stopFlag = false;
                            robot.setElivatorMotor(-1.0);
                            Log.d(TAG, "Elevator speed is " + elivatorSpeed);

                        } else if (gamepad2.dpad_left) {
                            robot.resetEncoderElivatorMotor();
                            Log.d(TAG, "Elevator position is " + robot.getEncoderPositionElivatorMotor());
                        } else {
                            robot.setElivatorMotor(0);
                            if (stopFlag == false) {
                                Log.d(TAG, "Elevator speed is 0");
                                stopFlag = true;
                            }
                        }
                    }
                    Log.d(TAG, " Elevat or thread is finished");
                    robot.setElivatorMotor(-0.1);
                    robot.ElivatorMotorUsingBrake();
                    };
            } catch (Exception e) {
                Log.d(TAG, "Elevator thread exception: " + e.getMessage());
            }
        }


    }*/
}





