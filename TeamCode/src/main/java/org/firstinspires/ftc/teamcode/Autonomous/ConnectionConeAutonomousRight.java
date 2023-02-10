package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.ConnectionHardware;
import org.firstinspires.ftc.teamcode.ImageProcessing.ImageProccessingOpenCV;

@Autonomous(name="RightConnectionConeAutonomous", group="Robot")
public class ConnectionConeAutonomousRight extends LinearOpMode {


    ConnectionHardware robot = new ConnectionHardware();
    ImageProccessingOpenCV imageProccessingOpenCV = new ImageProccessingOpenCV();
    private ElapsedTime runtime = new ElapsedTime();
    private String TAG = "AutonomousConnectionImageProccessing";
    private String TAG_E = "EncoderAutonomousConnectionImageProccessing";
    private ImageProccessingOpenCV.LabelProcessing labelProcessing = null;
    private final long sleepMsec = 200;

    static final double COUNTS_PER_MOTOR_REV = (134.4 * 4);  // PPR is 134.4 ; CPR = PPR * 4 for 1:20 Motor
    static final double DRIVE_GEAR_REDUCTION = (16.0 / 18.0);     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double SECOND_DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.45;
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.020;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    double armPower = 0.30;
    double elevatorPower = 0.30;
    double coneAngle = -42.5;
    final double timeout = 4;


    @Override
    public void runOpMode() {

        int counter = 0;

        Log.d(TAG, "Start Init");

        imageProccessingOpenCV.Init(hardwareMap);

        robot.init(hardwareMap);

        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();
        if (false == opModeIsActive())
        {
            Log.d(TAG, "Stop!!!");
            //imageProccessingOpenCV.StopRobot();
        }
        else
        {
            Log.d(TAG, "start...");
            imageProccessingOpenCV.LabelProcessingInit();
            runtime.reset();
            while ((labelProcessing == null) && (opModeIsActive()) && (!isStopRequested()) && (runtime.seconds() <= 5)) {
                labelProcessing = imageProccessingOpenCV.FindLabelProcessingOpenCV();
                Log.d(TAG, "Label Processing is = " + labelProcessing);
                Log.d(TAG, "timer is = " + runtime.seconds());
                if (labelProcessing != null) {
                    telemetry.addData("Label Processing is = ", labelProcessing);
                    telemetry.update();
                }
                connectionSleep(200);
            }
            if (labelProcessing == null) {
                labelProcessing = ImageProccessingOpenCV.LabelProcessing.TWO;
                telemetry.addLine("Image Processing not found label; select TWO");
                telemetry.update();
            }
            imageProccessingOpenCV.StopRobot();

            robot.setTuningClawMotor(0.05);
            robot.arm_motor.setTargetPosition(257);
            if (robot.arm_motor.getCurrentPosition() > 257) {
                armPower = armPower * -1;
            }
            robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm_motor.setPower(armPower);
            runtime.reset();
            Log.d(TAG_E, "armMotor target position is: " + robot.arm_motor.getTargetPosition());
            while ((opModeIsActive()) && (robot.ArmMotorIsBusy()) && (runtime.seconds() <= 3)) {
                Log.d(TAG_E, "arm encoder is " + robot.getEncoderPositionArmMotor());
            }
            Log.d(TAG_E, "time out arm end is: " + runtime.seconds());
            connectionSleep(sleepMsec);
            gyroTurn(TURN_SPEED, 90);
            connectionSleep(sleepMsec);
            gyroDrive(DRIVE_SPEED, 54, 90);
            connectionSleep(sleepMsec);
            gyroTurn(TURN_SPEED, 0);
            connectionSleep(sleepMsec);
            gyroDrive(DRIVE_SPEED, 110, 0);
            connectionSleep(sleepMsec);
            gyroTurn(TURN_SPEED, 0);
            connectionSleep(sleepMsec);
            gyroTurn(TURN_SPEED, coneAngle);
            connectionSleep(sleepMsec);
            gyroTurn(TURN_SPEED, coneAngle);
            connectionSleep(sleepMsec);
            robot.elevatorMotor.setTargetPosition(1828); //1828
            if (robot.elevatorMotor.getCurrentPosition() > 1828) { //1828
                elevatorPower = elevatorPower * -1;
            }
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevatorMotor.setPower(elevatorPower);
            runtime.reset();
            Log.d(TAG_E, "elevatorMotor target position is: " + robot.elevatorMotor.getTargetPosition());
            while ((opModeIsActive()) && (robot.ElivatorMotorIsBusy()) && (runtime.seconds() <= timeout)) {
                Log.d(TAG_E, "elevator encoder is " + robot.getEncoderPositionElivatorMotor());
            }
            Log.d(TAG_E, "time out elevator end is: " + runtime.seconds());
            connectionSleep(sleepMsec);
            gyroDrive(SECOND_DRIVE_SPEED, 9, coneAngle);
            connectionSleep(sleepMsec);
            //gyroTurn(SECOND_DRIVE_SPEED, coneAngle);
            //connectionSleep(sleepMsec);
            robot.elevatorMotor.setTargetPosition(1400);
            if (robot.elevatorMotor.getCurrentPosition() > 1400) {
                elevatorPower = elevatorPower * -1;
            }
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevatorMotor.setPower(elevatorPower);
            runtime.reset();
            Log.d(TAG_E, "elevatorMotor target position is: " + robot.elevatorMotor.getTargetPosition());
            while ((opModeIsActive()) && (robot.ElivatorMotorIsBusy()) && (runtime.seconds() <= timeout)) {
                Log.d(TAG_E, "elevator encoder is " + robot.getEncoderPositionElivatorMotor());
            }
            Log.d(TAG_E, "time out elevator end is: " + runtime.seconds());
            connectionSleep(sleepMsec);
            gyroTurn(SECOND_DRIVE_SPEED, coneAngle);
            robot.setCatchTheConeMotor(0.6);
            connectionSleep(sleepMsec);
            gyroDrive(DRIVE_SPEED, -7, coneAngle);
            connectionSleep(sleepMsec);
            robot.elevatorMotor.setTargetPosition(200);
            if (robot.elevatorMotor.getCurrentPosition() > 200) {
                elevatorPower = elevatorPower * -1;
            }
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.elevatorMotor.setPower(elevatorPower);
            runtime.reset();
            Log.d(TAG_E, "elevatorMotor target position is: " + robot.elevatorMotor.getTargetPosition());
            while ((opModeIsActive()) && (robot.ElivatorMotorIsBusy()) && (runtime.seconds() <= timeout)) {
                Log.d(TAG_E, "elevator encoder is " + robot.getEncoderPositionElivatorMotor());
            }
            Log.d(TAG_E, "time out elevator end is: " + runtime.seconds());
            connectionSleep(sleepMsec);
            robot.setCatchTheConeMotor(1.0);
            connectionSleep(sleepMsec);
            robot.setTuningClawMotor(0.25);
            robot.arm_motor.setTargetPosition(0);
            if (robot.arm_motor.getCurrentPosition() > 0) {
                armPower = armPower * -1;
            }
            robot.arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm_motor.setPower(armPower);
            Log.d(TAG_E, "armMotor target position is: " + robot.arm_motor.getTargetPosition());
            while ((opModeIsActive()) && (robot.ArmMotorIsBusy()) && (runtime.seconds() <= timeout)) {
                Log.d(TAG_E, "arm encoder is " + robot.getEncoderPositionArmMotor());
            }
            Log.d(TAG_E, "time out arm end is: " + runtime.seconds());
            if (labelProcessing == ImageProccessingOpenCV.LabelProcessing.ONE) {
                Log.d(TAG, "It is " + labelProcessing);
                telemetry.addLine("It is ONE");
                gyroTurn(TURN_SPEED, 0);
                connectionSleep(sleepMsec);
                gyroDrive(SECOND_DRIVE_SPEED, -20, 0);
                connectionSleep(sleepMsec);
                gyroTurn(TURN_SPEED, 0);


            } else if (labelProcessing == ImageProccessingOpenCV.LabelProcessing.TWO) {
                Log.d(TAG, "It is " + labelProcessing);
                telemetry.addLine("It is TWO");
                gyroTurn(TURN_SPEED, -90);
                connectionSleep(sleepMsec);
                gyroDrive(SECOND_DRIVE_SPEED, 51, -90);
                connectionSleep(sleepMsec);
                gyroTurn(TURN_SPEED, 0);
                connectionSleep(sleepMsec);
                gyroDrive(SECOND_DRIVE_SPEED, -20, 0);
                connectionSleep(sleepMsec);
                gyroTurn(TURN_SPEED, 0);

            } else {
                Log.d(TAG, "It is " + labelProcessing);
                telemetry.addLine("It is THREE");
                connectionSleep(sleepMsec);
                gyroTurn(TURN_SPEED, -90);
                connectionSleep(sleepMsec);
                gyroDrive(SECOND_DRIVE_SPEED, 108, -90);

            }

            while (opModeIsActive() && (!isStopRequested())) ;
        }
    }

    public void driveToTheSleeveParking() {
        if (labelProcessing == ImageProccessingOpenCV.LabelProcessing.ONE) {
            gyroTurn(0.45, 90);
            connectionSleep(500);
            gyroDrive(0.6, 47, 90);
            connectionSleep(500);
            gyroTurn(0.45, 0);
            connectionSleep(500);
            gyroDrive(0.6, 55, 0);
            connectionSleep(500);
            gyroTurn(0.45, 0);

        } else if (labelProcessing == ImageProccessingOpenCV.LabelProcessing.TWO) {

            gyroTurn(0.45, 90);
            connectionSleep(500);
            gyroDrive(0.6, 12, 90);
            connectionSleep(500);
            gyroTurn(0.45, 0);
            connectionSleep(500);
            gyroDrive(0.6, 55, 0);
            connectionSleep(500);
            gyroTurn(0.45, 0);
        } else {
            gyroTurn(0.45, -90);
            connectionSleep(500);
            gyroDrive(0.6, 30, -90);
            connectionSleep(500);
            gyroTurn(0.45, 0);
            connectionSleep(500);
            gyroDrive(0.6, 55, 0);
            connectionSleep(500);
            gyroTurn(0.45, 0);

        }
    }
    public void connectionSleep (long milliseconds)
    {
        if ((opModeIsActive()) && (!isStopRequested()))
        {
            sleep(milliseconds);
        }
        else
        {
            robot.setLeft_frontDrive(0);
            robot.setRight_frontDrive(0);
            robot.setLeft_backDrive(0);
            robot.setRight_backDrive(0);
        }
    }

    public void sideDriveImageProcessing(ImageProccessingOpenCV.LabelProcessing _labelProcessing) {
        if (_labelProcessing == ImageProccessingOpenCV.LabelProcessing.TWO) {
            sleep(500);
            gyroTurn(0.4, -90);
            sleep(500);
            gyroDrive(0.6, 47, -90);
            sleep(500);
            gyroTurn(0.4, 0);
            sleep(500);
            gyroTurn(0.4, 0);
        } else if (_labelProcessing == ImageProccessingOpenCV.LabelProcessing.THREE) {
            sleep(500);
            gyroTurn(0.4, -90);
            sleep(500);
            gyroDrive(0.6, 47, -90);
            sleep(500);
            gyroDrive(0.3, 52, -90);
            sleep(500);
            gyroTurn(0.4, 0);
            sleep(500);
            gyroTurn(0.4, 0);
        }
    }


    public void gyroTurn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        Log.d(TAG, "first turn, angle is" + angle);
        while ((opModeIsActive()) && (!isStopRequested()) && (!onHeading(speed, angle, P_TURN_COEFF, 2))) {
            // Updates telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        connectionSleep(100);
        Log.d(TAG, "second turn, angle is " + angle);
        while ((opModeIsActive()) && (!isStopRequested()) && (!onHeading((speed/2), angle, 0.04, 0.3))) {
            // Updates telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        Log.d(TAG, "finish turn, angle is " + angle);

        robot.setLeft_frontDrive(0);
        robot.setRight_frontDrive(0);
        robot.setLeft_backDrive(0);
        robot.setRight_backDrive(0);
        Log.d(TAG, "stop turn");

        Log.d(TAG, "The angle is " + robot.GetImuAngle());



    }


    boolean onHeading(double speed, double angle, double PCoeff, double heading_threshold) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= heading_threshold) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            if (Math.abs(rightSpeed) < 0.25)
            {
                if (rightSpeed < 0)
                {
                    rightSpeed = -0.25;
                }
                else
                {
                    rightSpeed = 0.25;
                }
            }
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.setLeft_frontDrive(leftSpeed);
        robot.setRight_frontDrive(rightSpeed);
        robot.setLeft_backDrive(leftSpeed);
        robot.setRight_backDrive(rightSpeed);

        // Display it for the driver.
        /*telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);*/

        Log.d(TAG, "Target" + " " + angle);
        Log.d(TAG, "Err/St" + " " + error + " " + steer);
        Log.d(TAG, "Speed" + " " + leftSpeed + " " + rightSpeed);
        Log.d(TAG, "" + robot.GetImuAngle());


        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;
        float gyroAngle = robot.GetImuAngle();
        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.GetImuAngle();
        Log.d(TAG, "target angle is " + targetAngle);
        Log.d(TAG, "angle is " + gyroAngle);
        while (robotError > 180 && (opModeIsActive()) && (!isStopRequested())) robotError -= 360;
        while (robotError <= -180 && (opModeIsActive()) && (!isStopRequested())) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroDrive(double speed, double distance, double angle) {

        int newLeftFTarget;
        int newLeftBTarget;
        int newRightFTarget;
        int newRightBTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        int positionLeftDriveF;
        int positionLeftDriveB;
        int positionRightDriveF;
        int positionRightDriveB;
        int rangLeftDriveF;
        int rangLeftDriveB;
        int rangRightDriveF;
        int rangRightDriveB;


        // Ensure that the opmode is still active
        if ((opModeIsActive()) && (!isStopRequested())) {

            robot.resetEncoderLeft_frontDrive();
            robot.resetEncoderRight_frontDrive();
            robot.resetEncoderLeft_backDrive();
            robot.resetEncoderRight_backDrive();

            robot.Left_frontDriveUsingEncoder();
            robot.Right_frontDriveUsingEncoder();
            robot.Left_backDriveUsingEncoder();
            robot.Right_backDriveUsingEncoder();


            distance = robot.cm_to_inch(distance);
            Log.d(TAG, "the distance is (inch) = " + distance);
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftFTarget = robot.getEncoderPositionLeft_frontDrive() + moveCounts;
            newRightFTarget = robot.getEncoderPositionRight_frontDrive() + moveCounts;
            newLeftBTarget = robot.getEncoderPositionLeft_backDrive() + moveCounts;
            newRightBTarget = robot.getEncoderPositionRight_backDrive() + moveCounts;

            Log.d(TAG, "newLeftFTarget is " + newLeftFTarget);
            Log.d(TAG, "newRightFTarget is " + newRightFTarget);
            Log.d(TAG, "newLeftBTarget is " + newLeftBTarget);
            Log.d(TAG, "newRightBTarget is " + newRightBTarget);


            // Set Target and Turn On RUN_TO_POSITION
            robot.Left_frontDriveSetTargetPosition(newLeftFTarget);
            robot.Right_frontDriveSetTargetPosition(newRightFTarget);
            robot.Left_backDriveSetTargetPosition(newLeftBTarget);
            robot.Right_backDriveSetTargetPosition(newRightBTarget);

            robot.Left_frontDriveToPosition();
            robot.Right_frontDriveToPosition();
            robot.Left_backDriveToPosition();
            robot.Right_backDriveToPosition();

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.setLeft_frontDrive(speed);
            robot.setRight_frontDrive(speed);
            robot.setLeft_backDrive(speed);
            robot.setRight_backDrive(speed);

            positionLeftDriveF = robot.getEncoderPositionLeft_frontDrive();
            positionRightDriveF = robot.getEncoderPositionRight_frontDrive();
            positionLeftDriveB = robot.getEncoderPositionLeft_backDrive();
            positionRightDriveB = robot.getEncoderPositionRight_backDrive();
            rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
            rangRightDriveF = Math.abs(newRightFTarget - positionRightDriveF);
            rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
            rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);


            // keep looping while we are still active, and BOTH motors are running.
            while ((opModeIsActive()) && (!isStopRequested()) &&
                    (robot.Left_frontDriveIsBusy() == true && robot.Right_frontDriveIsBusy() == true
                            && robot.Left_backDriveIsBusy() == true && robot.Right_backDriveIsBusy() == true)) {
                //(rangLeftDriveF > 150) && (rangRightDriveF > 150) && (rangLeftDriveB > 150) && (rangRightDriveB > 150))

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.setLeft_frontDrive(leftSpeed);
                    robot.setRight_frontDrive(rightSpeed);
                    robot.setLeft_backDrive(leftSpeed);
                    robot.setRight_backDrive(rightSpeed);

                    // Display drive status for the driver.
                /*telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDriveF.getCurrentPosition(),
                        robot.rightDriveF.getCurrentPosition());
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftDriveB.getCurrentPosition(),
                        robot.rightDriveB.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();*/

                    positionLeftDriveF = robot.getEncoderPositionLeft_frontDrive();
                    positionRightDriveF = robot.getEncoderPositionRight_frontDrive();
                    positionLeftDriveB = robot.getEncoderPositionLeft_backDrive();
                    positionRightDriveB = robot.getEncoderPositionRight_backDrive();
                    rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
                    rangRightDriveF = Math.abs(newRightFTarget - positionRightDriveF);
                    rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
                    rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);

                    Log.d(TAG, "Err/St" + " , " + error + " , " + steer);
                    Log.d(TAG, "Target" + " , " + newLeftFTarget + " , " + newRightFTarget + " , " + newLeftBTarget + " , " + newRightBTarget);
                    Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_frontDrive() + " , " + robot.getEncoderPositionRight_frontDrive());
                    Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_backDrive() + " , " + robot.getEncoderPositionRight_backDrive());
                    Log.d(TAG, "Speed" + " , " + leftSpeed + " , " + rightSpeed);

                }

                /*f (robot.Left_frontDriveIsBusy() == false) {
                    Log.d(TAG, "left front drive is not busy");
                }
                else if (robot.Right_frontDriveIsBusy() == false) {
                    Log.d(TAG, "right front drive is not busy");
                }
                else if (robot.Left_backDriveIsBusy() == false) {
                    Log.d(TAG, "left back drive is not busy");
                }
                else if (robot.Right_backDriveIsBusy() == false) {
                    Log.d(TAG, "right back drive is not busy");
                }*/

                // Stop all motion;
                robot.setLeft_frontDrive(0);
                robot.setRight_frontDrive(0);
                robot.setLeft_backDrive(0);
                robot.setRight_backDrive(0);
                Log.d(TAG, "stop drive");

                // Turn off RUN_TO_POSITION
                robot.Left_frontDriveUsingEncoder();
                robot.Right_frontDriveUsingEncoder();
                robot.Left_backDriveUsingEncoder();
                robot.Right_backDriveUsingEncoder();
            }
        else
        {
            robot.setLeft_frontDrive(0);
            robot.setRight_frontDrive(0);
            robot.setLeft_backDrive(0);
            robot.setRight_backDrive(0);
        }
    }

        public void sideDriveAutonomous ( double speed)
        {
            robot.sideDrive(speed);
            Log.d(TAG, "speed is " + speed);
            Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_frontDrive() + " , " + robot.getEncoderPositionRight_frontDrive());
            Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_backDrive() + " , " + robot.getEncoderPositionRight_backDrive());

        }
    }




