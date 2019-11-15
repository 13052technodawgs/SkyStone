package org.firstinspires.ftc.teamcode.jacobrefactor;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Implements debounced booleans to detect button presses and releases.
 *
 * Any "Pressed" method returns true only the first frame that a button
 * has been pressed. Conversely, any "Released" method returns true only
 * the first frame that a button has been released.
 *
 */
public class GamepadExtender {
    private Gamepad gamepad;

    private boolean aPressed,  bPressed,  xPressed,  yPressed,
                    aReleased, bReleased, xReleased, yReleased;

    private boolean dUpPressed,  dDownPressed,  dLeftPressed,  dRightPressed,
                    dUpReleased, dDownReleased, dLeftReleased, dRightReleased;

    private boolean lastA, lastB, lastX, lastY, lastUp, lastDown, lastLeft, lastRight;

    private boolean a, b, x, y, start, back, guide, leftBumper, rightBumper,
                    dPadUp, dPadDown, dPadLeft, dPadRight,
                    leftStickButton, rightStickButton;
    private double leftStickX, leftStickY, rightStickX, rightStickY;
    private double leftTrigger, rightTrigger;


    public GamepadExtender(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public void update(){
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        start = gamepad.start;
        back = gamepad.back;
        guide = gamepad.guide;
        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;
        dPadUp = gamepad.dpad_up;
        dPadDown = gamepad.dpad_down;
        dPadLeft = gamepad.dpad_left;
        dPadRight = gamepad.dpad_right;
        leftStickButton = gamepad.left_stick_button;
        rightStickButton = gamepad.right_stick_button;

        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;

        if(a){
            aReleased=false;
            aPressed=!lastA;
        }else{
            aPressed=false;
            aReleased=lastA;
        }
        if(b){
            bReleased=false;
            bPressed=!lastB;
        }else{
            bPressed=false;
            bReleased=lastB;
        }
        if(x){
            xReleased=false;
            xPressed=!lastX;
        }else{
            xPressed=false;
            xReleased=lastX;
        }
        if(y){
            yReleased=false;
            yPressed=!lastY;
        }else{
            yPressed=false;
            yReleased=lastY;
        }
        if(dPadUp){
            dUpReleased=false;
            dUpPressed=!lastUp;
        }else{
            dUpPressed=false;
            dUpReleased=lastUp;
        }
        if(dPadDown){
            dDownReleased=false;
            dDownPressed=!lastDown;
        }else{
            dDownPressed=false;
            dDownReleased=lastDown;
        }
        if(dPadLeft){
            dLeftReleased=false;
            dLeftPressed=!lastA;
        }else{
            dLeftPressed=false;
            dLeftReleased=lastLeft;
        }
        if(dPadRight){
            dRightReleased=false;
            dRightPressed=!lastRight;
        }else{
            dRightPressed=false;
            dRightReleased=lastRight;
        }

        lastA     = a;
        lastB     = b;
        lastX     = x;
        lastY     = y;
        lastUp    = dPadUp;
        lastDown  = dPadDown;
        lastLeft  = dPadLeft;
        lastRight = dPadRight;
    }

    public boolean aPressed() {
        return aPressed;
    }

    public boolean bPressed() {
        return bPressed;
    }

    public boolean xPressed() {
        return xPressed;
    }

    public boolean yPressed() {
        return yPressed;
    }

    public boolean aReleased() {
        return aReleased;
    }

    public boolean bReleased() {
        return bReleased;
    }

    public boolean xReleased() {
        return xReleased;
    }

    public boolean yReleased() {
        return yReleased;
    }

    public boolean dUpPressed() {
        return dUpPressed;
    }

    public boolean dDownPressed() {
        return dDownPressed;
    }

    public boolean dLeftPressed() {
        return dLeftPressed;
    }

    public boolean dRightPressed() {
        return dRightPressed;
    }

    public boolean dUpReleased() {
        return dUpReleased;
    }

    public boolean dDownReleased() {
        return dDownReleased;
    }

    public boolean dLeftReleased() {
        return dLeftReleased;
    }

    public boolean dRightReleased() {
        return dRightReleased;
    }

    public boolean getA() {
        return a;
    }

    public boolean getB() {
        return b;
    }

    public boolean getX() {
        return x;
    }

    public boolean getY() {
        return y;
    }

    public boolean getStart() {
        return start;
    }

    public boolean getBack() {
        return back;
    }

    public boolean getGuide() {
        return guide;
    }

    public boolean getLeftBumper() {
        return leftBumper;
    }

    public boolean getRightBumper() {
        return rightBumper;
    }

    public boolean getDPadUp() {
        return dPadUp;
    }

    public boolean getDPadDown() {
        return dPadDown;
    }

    public boolean getDPadLeft() {
        return dPadLeft;
    }

    public boolean getDPadRight() {
        return dPadRight;
    }

    public boolean getLeftStickButton() {
        return leftStickButton;
    }

    public boolean getRightStickButton() {
        return rightStickButton;
    }

    public double getLeftStickX() {
        return leftStickX;
    }

    public double getLeftStickY() {
        return leftStickY;
    }

    public double getRightStickX() {
        return rightStickX;
    }

    public double getRightStickY() {
        return rightStickY;
    }

    public double getLeftTrigger() {
        return leftTrigger;
    }

    public double getRightTrigger() {
        return rightTrigger;
    }
}
