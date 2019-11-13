package org.firstinspires.ftc.teamcode;

public enum RobotDirection {

    FORWARD(-1,1,-1,1),
    BACKWARD(1,-1,1,-1),
    RIGHT(-1,-1,1,1),
    LEFT(1,1,-1,-1);


    private final int[] matrix;

    RobotDirection(int... matrix){
        this.matrix = matrix;
    }

    public int FL(){
        return matrix[0];
    }

    public int FR() {
        return matrix[1];
    }

    public int BL() {
        return matrix[2];
    }

    public int BR(){
        return matrix[3];
    }


}
