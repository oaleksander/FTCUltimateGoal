package org.firstinspires.ftc.teamcode;

public class WoENregulators extends WoENrobot{

  /*  WoENrobot robot = new WoENrobot();
    private ElapsedTime loopTime = new ElapsedTime();

    static float targetHeading = 0.0f;


        public void GyroTurn(float angle)
        {
            final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 3;
            targetHeading = AngleTransform(targetHeading+angle);
            float angleError = 0, deltaError = 0, oldError = 0;
            loopTime.reset();
            oldError = AngleTransform(targetHeading - robot.getAngle());
            do{
                angleError = AngleTransform(targetHeading - robot.getAngle());
                deltaError = angleError - oldError;
                robot.tankTurn(angleError*Kp+deltaError*Kd);
                oldError = angleError;
            }while(Math.abs(angleError)>minError && loopTime.milliseconds()<maxTime);
            robot.stopMoveMotors();
        }


    public void encoderMoveForward(int distance)
    {
        distance *= robot.EncoderCountsPerCM*Math.sqrt(2);
        final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 20;
        int encoderError = 0, deltaError = 0, oldError = 0;
        robot.resetMoveEncoders();
        loopTime.reset();
        oldError = distance - (robot.leftDriveF.getCurrentPosition()+robot.rightDriveF.getCurrentPosition()
                              +robot.leftDriveR.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/4;
        do{
            encoderError = distance - (robot.leftDriveF.getCurrentPosition()+robot.rightDriveF.getCurrentPosition()
                           +robot.leftDriveR.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/4;
            deltaError = encoderError - oldError;
            robot.omniMoveYX(encoderError*Kp+deltaError*Kd, 0);
            oldError = encoderError;
        }while(Math.abs(encoderError)>minError && loopTime.milliseconds()<maxTime);
        robot.stopMoveMotors();
    }

    public void encoderMoveSideways(int distance)
    {
        distance *= robot.EncoderCountsPerCM*Math.sqrt(2);
        final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 20;
        int encoderError = 0, deltaError = 0, oldError = 0;
        robot.resetMoveEncoders();
        loopTime.reset();
        oldError = distance - (robot.leftDriveF.getCurrentPosition()-robot.rightDriveF.getCurrentPosition()
                -robot.leftDriveR.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/4;
        do{
            encoderError = (robot.leftDriveF.getCurrentPosition()-robot.rightDriveF.getCurrentPosition()
                           -robot.leftDriveR.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/4;
            deltaError = encoderError - oldError;
            robot.omniMoveYX(0, encoderError*Kp+deltaError*Kd);
            oldError = encoderError;
        }while(Math.abs(encoderError)>minError && loopTime.milliseconds()<maxTime);
        robot.stopMoveMotors();
    }

    public void encoderMoveDiagonalRight(int distance)
    {
        distance *= robot.EncoderCountsPerCM;
        final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 20;
        int encoderError = 0, deltaError = 0, oldError = 0;
        robot.resetMoveEncoders();
        robot.rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); robot.leftDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loopTime.reset();
        oldError = distance - (robot.leftDriveF.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/2;
        do{
            encoderError = distance - (robot.leftDriveF.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/2;
            deltaError = encoderError - oldError;
            robot.omniMoveYX(encoderError*Kp+deltaError*Kd, encoderError*Kp+deltaError*Kd);
            oldError = encoderError;
        }while(Math.abs(encoderError)>minError && loopTime.milliseconds()<maxTime);
        robot.rightDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); robot.leftDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.stopMoveMotors();
    }

    public void encoderMoveDiagonalLeft(int distance)
    {
        distance *= robot.EncoderCountsPerCM;
        final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 20;
        int encoderError = 0, deltaError = 0, oldError = 0;
        robot.resetMoveEncoders();
        robot.leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); robot.rightDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        loopTime.reset();
        oldError = distance - (robot.rightDriveF.getCurrentPosition()+robot.leftDriveR.getCurrentPosition())/2;
        do{
            encoderError = distance - (robot.rightDriveF.getCurrentPosition()+robot.leftDriveR.getCurrentPosition())/2;
            deltaError = encoderError - oldError;
            robot.omniMoveYX(encoderError*Kp+deltaError*Kd, -encoderError*Kp+deltaError*Kd);
            oldError = encoderError;
        }while(Math.abs(encoderError)>minError && loopTime.milliseconds()<maxTime);
        robot.leftDriveF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); robot.rightDriveR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.stopMoveMotors();
    }

    public void encoderMovePolar(double heading, int distance)
    {
        heading = AngleTransformPI(Math.toRadians(heading));
        distance *= robot.EncoderCountsPerCM;
        final double Kp = 0.02, Kd = 0.01, maxTime = 4000, minError = 20;
        int encoderError = 0, deltaError = 0, oldError = 0;
        double Regulator = 0, Ymovement, Xmovement = 0;
        robot.resetMoveEncoders();
        oldError = distance - (int)(((robot.leftDriveF.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/2)*Math.cos(heading-Math.PI/4)
                                  + ((robot.rightDriveF.getCurrentPosition()+robot.leftDriveR.getCurrentPosition())/2)*Math.cos(heading+Math.PI/4));
        do{
            encoderError = distance - (int)(((robot.leftDriveF.getCurrentPosition()+robot.rightDriveR.getCurrentPosition())/2)*Math.cos(heading-Math.PI/4)
                                          + ((robot.rightDriveF.getCurrentPosition()+robot.leftDriveR.getCurrentPosition())/2)*Math.cos(heading+Math.PI/4));
            deltaError = encoderError - oldError;
            Regulator = encoderError*Kp+deltaError*Kd;
            robot.omniMove(heading, Regulator);
            oldError = encoderError;
        }while(Math.abs(encoderError)>minError && loopTime.milliseconds()<maxTime);
    }

    public void toPoint(float Y, float X)
    {

        robot.omniMove(float Y, float X);
    } */
}