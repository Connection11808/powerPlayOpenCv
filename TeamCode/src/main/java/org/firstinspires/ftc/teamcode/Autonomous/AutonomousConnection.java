package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.ConnectionHardware;
import org.firstinspires.ftc.teamcode.ImageProcessing.ImageProcessingConnection;

@Autonomous(name="Autonomous", group="Robot")
public class AutonomousConnection extends LinearOpMode {

    ImageProcessingConnection imageProcessingConnection = new ImageProcessingConnection();
    ConnectionHardware robot = new ConnectionHardware();
    private ElapsedTime runtime = new ElapsedTime();
    ImageProcessingConnection.LabelProcessing labelProcessing;
    private String TAG = "AutonomousConnection";


    static final double COUNTS_PER_MOTOR_REV = (134.4 * 4);  // PPR is 134.4 ; CPR = PPR * 4 for 1:20 Motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.03;     // Larger is more responsive, but also less stable
    static final double HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro



    @Override
    public void runOpMode() {

        int counter = 0;

        Log.d(TAG, "Start Init");
        imageProcessingConnection.InitImageProcessing(hardwareMap);


        robot.init(hardwareMap);

        telemetry.addData(">", "Robot Ready");
        telemetry.update();

        waitForStart();


        labelProcessing = imageProcessingConnection.FindLabelProcessing();



        while ((labelProcessing == null) && (counter < 10)) {
             sleep(35);
             labelProcessing = imageProcessingConnection.FindLabelProcessing();
             counter++;
        }

        Log.d(TAG, "counter =" + counter);
        if (labelProcessing == null)
        {
            telemetry.addLine("The label processing is null");
            telemetry.update();
            Log.d(TAG,"The label processing is null");
            labelProcessing = ImageProcessingConnection.LabelProcessing.BOLT;
        }




        /*gyroDrive(0.5, 80, 0);
        gyroTurn(0.4, -90);
        gyroDrive(0.5, 145, -90);
        gyroTurn(0.4, -180);
        gyroDrive(0.5, 92, -180);
        gyroTurn(0.4, -270);
        gyroDrive(0.5, 142, -270);
        gyroTurn(0.4, 0);
        gyroTurn(0.4, 0);*/


        if (labelProcessing == ImageProcessingConnection.LabelProcessing.BOLT)
        {
            Log.d(TAG, "It is " + labelProcessing);
            telemetry.addLine("It is Bolt");
            startDrive();
            sideDriveImageProcessing(labelProcessing);
        }
        else if (labelProcessing == ImageProcessingConnection.LabelProcessing.BULB)
        {
            Log.d(TAG, "It is " + labelProcessing);
            telemetry.addLine("It is Bulb");
            startDrive();
            sideDriveImageProcessing(labelProcessing);

        }
        else {
            Log.d(TAG, "It is " + labelProcessing);
            telemetry.addLine("It is Panel");
            startDrive();
            sideDriveImageProcessing(labelProcessing);

        }
    }
    public void startDrive ()
    {
        sideDriveAutonomous(0.5, 120, 0);
        gyroTurn(0.4, 0);
        gyroDrive(0.3, 60, 0);
        gyroTurn(0.4, 0);
    }

    public void sideDriveImageProcessing(ImageProcessingConnection.LabelProcessing _labelProcessing)
    {
        if (_labelProcessing == ImageProcessingConnection.LabelProcessing.BULB)
        {
            gyroTurn(0.4, -90);
            gyroDrive(0.3, 52, -90);
            sleep(500);
            gyroTurn(0.4, 0);
            sleep(500);
            gyroTurn(0.4, 0);
        }
        else if (_labelProcessing == ImageProcessingConnection.LabelProcessing.PANEL)
        {
            gyroTurn(0.4, -90);
            gyroDrive(0.3, 95, -90);
            sleep(500);
            gyroTurn(0.4, 0);
            sleep(500);
            gyroTurn(0.4, 0);
        }
    }


    public void gyroTurn (double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, HEADING_THRESHOLD)) {
            // Updates telemetry & Allow time for other processes to run.
            telemetry.update();
        }
        Log.d(TAG, "The angle is " + robot.GetImuAngle());

    }


        boolean onHeading (double speed, double angle, double PCoeff, double heading_threshold) {
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


        public double getError ( double targetAngle){

            double robotError;

            // calculate error in -179 to +180 range  (
            robotError = targetAngle - robot.GetImuAngle();
            while (robotError > 180) robotError -= 360;
            while (robotError <= -180) robotError += 360;
            return robotError;
        }

        public double getSteer ( double error, double PCoeff){
            return Range.clip(error * PCoeff, -1, 1);
        }

        public void gyroDrive ( double speed, double distance, double angle){

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
            if (opModeIsActive()) {

                robot.resetEncoderLeft_frontDrive();
                robot.resetEncoderRight_frontDrive();
                robot.resetEncoderLeft_backDrive();
                robot.resetEncoderRight_backDrive();

                distance = robot.cm_to_inch(distance);
                Log.d(TAG, "the distance is (inch) = " + distance);
                // Determine new target position, and pass to motor controller
                moveCounts = (int) (distance * COUNTS_PER_INCH);
                newLeftFTarget = robot.getEncoderPositionLeft_frontDrive() + moveCounts;
                newRightFTarget = robot.getEncoderPositionRight_frontDrive() + moveCounts;
                newLeftBTarget = robot.getEncoderPositionLeft_backDrive() + moveCounts;
                newRightBTarget = robot.getEncoderPositionRight_backDrive() + moveCounts;

                // Set Target and Turn On RUN_TO_POSITION
                //robot.Left_frontDriveSetTargetPosition(newLeftFTarget);
                //robot.Right_frontDriveSetTargetPosition(newRightFTarget);
                //robot.Left_backDriveSetTargetPosition(newLeftBTarget);
                //robot.Right_backDriveSetTargetPosition(newRightBTarget);

                robot.Left_frontDriveUsingEncoder();
                robot.Right_frontDriveUsingEncoder();
                robot.Left_backDriveUsingEncoder();
                robot.Right_backDriveUsingEncoder();

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
                while (opModeIsActive() &&
                        //(robot.Left_frontDriveIsBusy() && robot.Right_frontDriveIsBusy() && robot.Left_backDriveIsBusy() && robot.Right_backDriveIsBusy())) {
                        (rangLeftDriveF > 150) && (rangRightDriveF > 150) && (rangLeftDriveB > 150) /*&& (rangRightDriveB > 150)*/) {

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

                // Stop all motion;
                robot.setLeft_frontDrive(0);
                robot.setRight_frontDrive(0);
                robot.setLeft_backDrive(0);
                robot.setRight_backDrive(0);


                // Turn off RUN_TO_POSITION
                robot.Left_frontDriveUsingEncoder();
                robot.Right_frontDriveUsingEncoder();
                robot.Left_backDriveUsingEncoder();
                robot.Right_backDriveUsingEncoder();
            }
        }
    public void sideDriveAutonomous ( double speed, double distance, double angle){

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
        if (opModeIsActive()) {

            robot.resetEncoderLeft_frontDrive();
            robot.resetEncoderRight_frontDrive();
            robot.resetEncoderLeft_backDrive();
            robot.resetEncoderRight_backDrive();

            Log.d(TAG, "start sideDriveAutonomous");
            distance = robot.cm_to_inch(distance);
            Log.d(TAG, "the distance is (inch) = " + distance);
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftFTarget = robot.getEncoderPositionLeft_frontDrive() + moveCounts;
            newRightFTarget = robot.getEncoderPositionRight_frontDrive() + moveCounts;
            newLeftBTarget = robot.getEncoderPositionLeft_backDrive() + moveCounts;
            newRightBTarget = robot.getEncoderPositionRight_backDrive() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            //robot.Left_frontDriveSetTargetPosition(newLeftFTarget);
            //robot.Right_frontDriveSetTargetPosition(newRightFTarget);
            //robot.Left_backDriveSetTargetPosition(newLeftBTarget);
            //robot.Right_backDriveSetTargetPosition(newRightBTarget);

            robot.Left_frontDriveUsingEncoder();
            robot.Right_frontDriveUsingEncoder();
            robot.Left_backDriveUsingEncoder();
            robot.Right_backDriveUsingEncoder();

            // start motion.
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.setLeft_frontDrive(-speed); // 75
            robot.setRight_frontDrive(speed); // 0.4
            robot.setLeft_backDrive(speed);   // 0.63
            robot.setRight_backDrive(-speed); // 0.48

            Log.d(TAG, "The speed is " + speed);

            positionLeftDriveF = robot.getEncoderPositionLeft_frontDrive();
            positionRightDriveF = robot.getEncoderPositionRight_frontDrive();
            positionLeftDriveB = robot.getEncoderPositionLeft_backDrive();
            positionRightDriveB = robot.getEncoderPositionRight_backDrive();
            rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
            rangRightDriveF = Math.abs(newRightFTarget - positionRightDriveF);
            rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
            rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    //(robot.Left_frontDriveIsBusy() && robot.Right_frontDriveIsBusy() && robot.Left_backDriveIsBusy() && robot.Right_backDriveIsBusy())) {
                    (rangLeftDriveF > 150) && (rangRightDriveF > 150) && (rangLeftDriveB > 150) /*&& (rangRightDriveB > 150)*/) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.setLeft_frontDrive(-leftSpeed);
                robot.setRight_frontDrive(rightSpeed);
                robot.setLeft_backDrive(leftSpeed);
                robot.setRight_backDrive(-rightSpeed);

                positionLeftDriveF = robot.getEncoderPositionLeft_frontDrive();
                positionRightDriveF = robot.getEncoderPositionRight_frontDrive();
                positionLeftDriveB = robot.getEncoderPositionLeft_backDrive();
                positionRightDriveB = robot.getEncoderPositionRight_backDrive();
                rangLeftDriveF = Math.abs(newLeftFTarget - positionLeftDriveF);
                rangRightDriveF = Math.abs(newRightFTarget - positionRightDriveF);
                rangLeftDriveB = Math.abs(newLeftBTarget - positionLeftDriveB);
                rangRightDriveB = Math.abs(newRightBTarget - positionRightDriveB);

                Log.d(TAG, "Target" + " , " + newLeftFTarget + " " + newRightFTarget + " , " + newLeftBTarget + " , " + newRightBTarget);
                Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_frontDrive() + " , " + robot.getEncoderPositionRight_frontDrive());
                Log.d(TAG, "Actual" + " , " + robot.getEncoderPositionLeft_backDrive() + " , " + robot.getEncoderPositionRight_backDrive());

            }

            // Stop all motion;
            robot.setLeft_frontDrive(0);
            robot.setRight_frontDrive(0);
            robot.setLeft_backDrive(0);
            robot.setRight_backDrive(0);


            // Turn off RUN_TO_POSITION
            robot.Left_frontDriveUsingEncoder();
            robot.Right_frontDriveUsingEncoder();
            robot.Left_backDriveUsingEncoder();
            robot.Right_backDriveUsingEncoder();
        }
    }



    }


