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

package org.firstinspires.ftc.teamcode.Test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Hardware.ConnectionHardware;


@TeleOp(name = "ConnectionUnitTest", group = "Sensor")
public class ConnectionUnitTest extends LinearOpMode {

    ConnectionHardware connectionRobot = new ConnectionHardware();
    String TAG = "unittest";
    private double MotorSpeed = 0.8;

    @Override
    public void runOpMode() {

        connectionRobot.init(hardwareMap);

        connectionRobot.resetEncoderLeft_frontDrive();
        connectionRobot.resetEncoderLeft_backDrive();
        connectionRobot.resetEncoderRight_frontDrive();
        connectionRobot.resetEncoderRight_backDrive();


        Log.d(TAG, "Finish Init");
        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            if (gamepad1.dpad_up == true)
            {
                connectionRobot.setLeft_frontDrive(MotorSpeed);
                connectionRobot.setLeft_backDrive(MotorSpeed);
                connectionRobot.setRight_frontDrive(MotorSpeed);
                connectionRobot.setRight_backDrive(MotorSpeed);
                connectionRobot.getEncoderPositionLeft_frontDrive();
                connectionRobot.getEncoderPositionLeft_backDrive();
                connectionRobot.getEncoderPositionRight_frontDrive();
                connectionRobot.getEncoderPositionRight_backDrive();
                Log.d(TAG, "left_frontDrive position is " + connectionRobot.getEncoderPositionLeft_frontDrive());
                Log.d(TAG, "left_backDrive position is " + connectionRobot.getEncoderPositionLeft_backDrive());
                Log.d(TAG, "right_frontDrive position is " + connectionRobot.getEncoderPositionRight_frontDrive());
                Log.d(TAG, "right_backDrive position is " + connectionRobot.getEncoderPositionRight_backDrive());
            }
            else if (gamepad1.dpad_right == true)
            {
                connectionRobot.sideDrive(MotorSpeed);
                connectionRobot.getEncoderPositionLeft_frontDrive();
                connectionRobot.getEncoderPositionLeft_backDrive();
                connectionRobot.getEncoderPositionRight_frontDrive();
                connectionRobot.getEncoderPositionRight_backDrive();
                Log.d(TAG, "left_frontDrive position is " + connectionRobot.getEncoderPositionLeft_frontDrive());
                Log.d(TAG, "left_backDrive position is " + connectionRobot.getEncoderPositionLeft_backDrive());
                Log.d(TAG, "right_frontDrive position is " + connectionRobot.getEncoderPositionRight_frontDrive());
                Log.d(TAG, "right_backDrive position is " + connectionRobot.getEncoderPositionRight_backDrive());
            }
            else if (gamepad1.dpad_down == true)
            {
                connectionRobot.sideDrive(-MotorSpeed);
                connectionRobot.getEncoderPositionLeft_frontDrive();
                connectionRobot.getEncoderPositionLeft_backDrive();
                connectionRobot.getEncoderPositionRight_frontDrive();
                connectionRobot.getEncoderPositionRight_backDrive();
                Log.d(TAG, "left_frontDrive position is " + connectionRobot.getEncoderPositionLeft_frontDrive());
                Log.d(TAG, "left_backDrive position is " + connectionRobot.getEncoderPositionLeft_backDrive());
                Log.d(TAG, "right_frontDrive position is " + connectionRobot.getEncoderPositionRight_frontDrive());
                Log.d(TAG, "right_backDrive position is " + connectionRobot.getEncoderPositionRight_backDrive());
            }
            else {
                if (gamepad1.y == true) {
                    connectionRobot.setLeft_frontDrive(MotorSpeed);
                    connectionRobot.getEncoderPositionLeft_frontDrive();
                    Log.d(TAG, "left_frontDrive position is " + connectionRobot.getEncoderPositionLeft_frontDrive());

                } else {
                    connectionRobot.setLeft_frontDrive(0);
                }
                if (gamepad1.a == true) {
                    connectionRobot.setLeft_backDrive(MotorSpeed);
                    connectionRobot.getEncoderPositionLeft_backDrive();
                    Log.d(TAG, "left_backDrive position is " + connectionRobot.getEncoderPositionLeft_backDrive());
                } else {
                    connectionRobot.setLeft_backDrive(0);
                }
                if (gamepad1.b == true) {
                    connectionRobot.setRight_frontDrive(MotorSpeed);
                    connectionRobot.getEncoderPositionRight_frontDrive();
                    Log.d(TAG, "right_frontDrive position is " + connectionRobot.getEncoderPositionRight_frontDrive());
                } else {
                    connectionRobot.setRight_frontDrive(0);
                }
                if (gamepad1.x == true) {
                    connectionRobot.setRight_backDrive(MotorSpeed);
                    connectionRobot.getEncoderPositionRight_backDrive();
                    Log.d(TAG, "right_backDrive position is " + connectionRobot.getEncoderPositionRight_backDrive());
                } else {
                    connectionRobot.setRight_backDrive(0);
                }
            }

            if (gamepad1.right_bumper == true) {
                MotorSpeed = -MotorSpeed;
                Log.d(TAG, "The speed is = " + MotorSpeed);
            }
            if (gamepad1.left_bumper == true)
            {
                connectionRobot.resetEncoderLeft_frontDrive();
                connectionRobot.resetEncoderLeft_backDrive();
                connectionRobot.resetEncoderRight_frontDrive();
                connectionRobot.resetEncoderRight_backDrive();
                connectionRobot.getEncoderPositionLeft_frontDrive();
                connectionRobot.getEncoderPositionLeft_backDrive();
                connectionRobot.getEncoderPositionRight_frontDrive();
                connectionRobot.getEncoderPositionRight_backDrive();
                Log.d(TAG, "left_frontDrive position is " + connectionRobot.getEncoderPositionLeft_frontDrive());
                Log.d(TAG, "left_backDrive position is " + connectionRobot.getEncoderPositionLeft_backDrive());
                Log.d(TAG, "right_frontDrive position is " + connectionRobot.getEncoderPositionRight_frontDrive());
                Log.d(TAG, "right_backDrive position is " + connectionRobot.getEncoderPositionRight_backDrive());
            }

            if (gamepad1.right_trigger > 0.5)
            {
                connectionRobot.setElivatorMotor(0.1);
                Log.d(TAG, "Encoder position of ElivatorMotor is " + connectionRobot.getEncoderPositionElivatorMotor());
            }
            else if (gamepad1.left_trigger > 0.5)
            {
                connectionRobot.setElivatorMotor(-0.1);
                Log.d(TAG, "Encoder position of ElivatorMotor is " + connectionRobot.getEncoderPositionElivatorMotor());

            }
            else {
                connectionRobot.setElivatorMotor(0);
                //robot.ElivatorMotorUsingBrake();
            }
            if (gamepad1.dpad_left == true)
            {
                connectionRobot.resetEncoderElivatorMotor();
                Log.d(TAG, "Encoder position of ElivatorMotor is " + connectionRobot.getEncoderPositionElivatorMotor());
            }





        }
    }
}
