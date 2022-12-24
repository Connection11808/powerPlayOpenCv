/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Hardware;


import static org.checkerframework.checker.units.UnitsTools.h;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class ConnectionHardware {

    private String TAG = "ConnectionHardware";


    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor left_frontDrive = null;
    public DcMotor left_backDrive = null;
    public DcMotor right_frontDrive = null;
    public DcMotor right_backDrive = null;
    public DcMotor elevatorMotor = null;
    public Servo catchTheCone = null;



    private BNO055IMU imu;

    // Define a constructor that allows the OpMode to pass a reference to itself.

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */

    public void init(HardwareMap hardwareMapConnection) {
        left_frontDrive = hardwareMapConnection.get(DcMotor.class, "LFD");
        left_backDrive = hardwareMapConnection.get(DcMotor.class, "LBD");
        right_frontDrive = hardwareMapConnection.get(DcMotor.class, "RFD");
        right_backDrive = hardwareMapConnection.get(DcMotor.class, "RBD");
        elevatorMotor = hardwareMapConnection.get(DcMotor.class, "EM");
        catchTheCone = hardwareMapConnection.get(Servo.class, "CTC");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_frontDrive.setDirection(DcMotor.Direction.FORWARD);
        left_backDrive.setDirection(DcMotor.Direction.FORWARD);
        right_frontDrive.setDirection(DcMotor.Direction.REVERSE);
        right_backDrive.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_frontDrive.setPower(0);
        left_backDrive.setPower(0);
        right_frontDrive.setPower(0);
        right_backDrive.setPower(0);
        elevatorMotor.setPower(0);
        catchTheCone.setPosition(0.75);


        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMapConnection.get(BNO055IMU.class, "imu ");
        boolean sucses = initImu();
        if (sucses == false)
        {
            imu = hardwareMapConnection.get(BNO055IMU.class, "imu1");
            initImu();
        }
    }


    public void sideDrive(double speed){
        right_frontDrive.setPower(speed);
        right_backDrive.setPower(-speed * 0.5);
        left_frontDrive.setPower(-speed * 0.5);
        left_backDrive.setPower(speed);
    }

    private boolean initImu(){
        boolean sucses;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        sucses = imu.initialize(parameters);
        return sucses;


    }

    public float GetImuAngle(){

        Orientation angles =imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    public double cm_to_inch (double cm)
    {
        double inch = cm * 0.393701;
        return(inch);

    }


    public void setLeft_frontDrive (double power) {

        left_frontDrive.setPower(power);
    }
    public void setLeft_backDrive (double power) {
        left_backDrive.setPower(power);
    }
    public void setRight_frontDrive (double power) {
        right_frontDrive.setPower(power);
    }
    public void setRight_backDrive (double power) {
        right_backDrive.setPower(power);
    }
    public void setElivatorMotor (double power) {
        elevatorMotor.setPower(power);
    }
    public void setCatchTheConeMotor (double position)
    {
        catchTheCone.setPosition(position);
    }
    public int getEncoderPositionLeft_frontDrive ()
    {
        int position;
        position = left_frontDrive.getCurrentPosition();
        return position;

    }
    public int getEncoderPositionLeft_backDrive ()
    {
        int position;
        position = left_backDrive.getCurrentPosition();
        return position;

    }
    public int getEncoderPositionRight_frontDrive ()
    {
        int position;
        position = right_frontDrive.getCurrentPosition();
        return position;

    }
    public int getEncoderPositionRight_backDrive ()
    {
        int position;
        position = right_backDrive.getCurrentPosition();
        return position;

    }
    public int getEncoderPositionElivatorMotor ()
    {
        int position;
        position = elevatorMotor.getCurrentPosition();
        return position;

    }
    public void resetEncoderLeft_frontDrive ()
    {
        left_frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoderLeft_backDrive ()
    {
        left_backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoderRight_frontDrive ()
    {
        right_frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_frontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoderRight_backDrive ()
    {
        right_backDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_backDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void resetEncoderElivatorMotor ()
    {
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void Left_frontDriveUsingEncoder ()
    {
        left_frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Left_backDriveUsingEncoder ()
    {
        left_backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Right_frontDriveUsingEncoder ()
    {
        right_frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Right_backDriveUsingEncoder ()
    {
        right_backDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void ElivatorMotorUsingEncoder ()
    {
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void Left_frontDriveToPosition()
    {
        left_frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Left_backDriveToPosition()
    {
        left_backDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Right_frontDriveToPosition()
    {
        right_frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Right_backDriveToPosition()
    {
        right_backDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void ElivatorMotorToPosition()
    {
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void Left_frontDriveSetTargetPosition(int position)
    {
        left_frontDrive.setTargetPosition(position);
    }
    public void Right_frontDriveSetTargetPosition(int position)
    {
        right_frontDrive.setTargetPosition(position);
    }
    public void Left_backDriveSetTargetPosition(int position)
    {
        left_backDrive.setTargetPosition(position);
    }
    public void Right_backDriveSetTargetPosition(int position)
    {
        right_backDrive.setTargetPosition(position);
    }
    public void ElivatorMotorSetTargetPosition(int position)
    {
        elevatorMotor.setTargetPosition(position);
    }
    public void ElivatorMotorUsingBrake()
    {
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public boolean Left_frontDriveIsBusy()
    {
        boolean isBusy;
        isBusy = left_frontDrive.isBusy();
        return isBusy;
    }
    public boolean Left_backDriveIsBusy()
    {
        boolean isBusy;
        isBusy = left_backDrive.isBusy();
        return isBusy;
    }
    public boolean Right_frontDriveIsBusy()
    {
        boolean isBusy;
        isBusy = right_frontDrive.isBusy();
        return isBusy;
    }
    public boolean Right_backDriveIsBusy()
    {
        boolean isBusy;
        isBusy = right_backDrive.isBusy();
        return isBusy;
    }
    public boolean ElivatorMotorIsBusy()
    {
        boolean isBusy;
        isBusy = elevatorMotor.isBusy();
        return isBusy;
    }




}
