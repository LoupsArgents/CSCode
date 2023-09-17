package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.DoubleSupplier;
/*
* Copyright (c) 2009-2019 FIRST All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted
* provided that the following conditions are met:
Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or other materials
* provided with the distribution.
Neither the name of the FIRST nor the names of its contributors may be used to endorse or promote
* products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY FIRST AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY NONINFRINGEMENT AND
* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FIRST OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
* NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/
public class TwoWheelOdometry extends Odometry {

    private double prevForwardEncoder, prevHorizontalEncoder;
    private Rotation2d previousAngle;
    private double centerWheelOffset;

    // the suppliers
    DoubleSupplier m_forward, m_horizontal;
    IMU imu;

    public TwoWheelOdometry(DoubleSupplier forwardEncoder,
                             DoubleSupplier horizontalEncoder, IMU imu, double centerWheelOffset) {
        this(centerWheelOffset); //creates an instance via the third constructor
        m_forward = forwardEncoder;
        m_horizontal = horizontalEncoder;
        this.imu = imu;
    }

    public TwoWheelOdometry(Pose2d initialPose, double centerWheelOffset) {
        super(initialPose); //creates an instance of class Odometry
        //OK so their class odometry *assumes* the existence of trackWidth
        //but is it actually used for anything, or can I just set it to 0 or 18 or whatever without consequence
        //given that I won't be using it here?
        //it's not used anywhere in odometry, so it won't be essential - it just needs to exist
        previousAngle = initialPose.getRotation();
        this.centerWheelOffset = centerWheelOffset;
    }

    public TwoWheelOdometry(double centerWheelOffset) {
        this(new Pose2d(), centerWheelOffset); //creates an instance via the second constructor
    }

    /**
     * This handles all the calculations for you.
     */
    @Override
    public void updatePose() {
        update(m_forward.getAsDouble(), m_horizontal.getAsDouble());
    }

    @Override
    public void updatePose(Pose2d pose) {
        previousAngle = pose.getRotation();
        robotPose = pose;
        prevForwardEncoder = 0;
        prevHorizontalEncoder = 0;
    }

    public void update(double forwardEncoderPos, double horizontalEncoderPos) {
        double deltaForwardEncoder = forwardEncoderPos - prevForwardEncoder;
        double deltaHorizontalEncoder = horizontalEncoderPos - prevHorizontalEncoder;
        Rotation2d angle = previousAngle.plus(Rotation2d.fromDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        //the above, the only use of trackWidth left, is to figure out the angle
        //so what does that actually do? says add this new thing to previousAngle
        //the only bit I need to change is what "this new thing" is
        //oh yeah we should probably take in an imu in the constructors. yeah.
        //ok now we've done that.
        prevForwardEncoder = forwardEncoderPos;
        prevHorizontalEncoder = horizontalEncoderPos;

        double dw = (angle.minus(previousAngle).getRadians());

        double dx = deltaForwardEncoder;
        double dy = deltaHorizontalEncoder - (centerWheelOffset * dw);

        Twist2d twist2d = new Twist2d(dx, dy, dw);

        Pose2d newPose = robotPose.exp(twist2d);

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

}