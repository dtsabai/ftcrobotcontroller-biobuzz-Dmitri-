package com.qualcomm.robotcore.hardware;
public interface DcMotor {
    enum Direction { FORWARD, REVERSE }
    enum RunMode { RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION, STOP_AND_RESET_ENCODER }
    enum ZeroPowerBehavior { BRAKE, FLOAT }
    void setDirection(Direction d);
    void setMode(RunMode m);
    void setZeroPowerBehavior(ZeroPowerBehavior z);
    void setPower(double p);
}
