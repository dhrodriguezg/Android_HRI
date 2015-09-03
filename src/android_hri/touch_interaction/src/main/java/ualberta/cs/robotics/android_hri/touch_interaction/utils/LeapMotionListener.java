package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import android.app.Activity;
import android.util.Log;

import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.FingerList;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.Listener;
import com.leapmotion.leap.Vector;


public class LeapMotionListener extends Listener {

    private static final String TAG = "LeapMotionListener";
    private static final float MIN_POS_X = -200;
    private static final float MIN_POS_Y = 0;
    private static final float MIN_POS_Z = -200;
    private static final float MAX_POS_X =  200;
    private static final float MAX_POS_Y =  400;
    private static final float MAX_POS_Z =  200;
    private static final float MIN_GRAP =  30;
    private static final float MAX_GRAP =  60;
    private static final float OPEN_THRESHOLD =  0.1875f;

    private LeapMotionFrameListener mListener;
    private Activity mActivity;

    private boolean rightHanded;

    public LeapMotionListener(Activity activity, LeapMotionFrameListener listener) {
        mActivity = activity;
        mListener = listener;
        rightHanded = true;
    }

    public void onInit(Controller controller) {
        Log.d(TAG, "Initialized");
    }

    public void onConnect(Controller controller) {
        mListener.onUpdateMsg("Connected");
    }

    public void onDisconnect(Controller controller) {
        mListener.onUpdateMsg("Disconnected");
    }

    public void onExit(Controller controller) {
        mListener.onUpdateMsg("Exited");
    }

    public void onFrame(Controller controller) {
        StringBuffer msg = new StringBuffer();
        Frame frame = controller.frame();
        boolean isRight = false;
        boolean isLeft = false;
        msg.append(String.format("FPS: %.2f", frame.currentFramesPerSecond()));
        msg.append(String.format("\nHands: %d, fingers: %d", frame.hands().count(), frame.fingers().count()));
        if (!frame.hands().isEmpty()) {
            mListener.onHands(true);
            for (Hand hand : frame.hands() ){
                if(!hand.isValid())
                    return;
                if(rightHanded){
                    if(hand.isRight()){
                        msg.append("\n   Action Hand (Right) : ");
                        rightHand(hand, msg);
                        isRight = true;
                    }else if(hand.isLeft()){
                        msg.append("\n   Task Hand(Left): ");
                        leftHand(hand, msg);
                        isLeft = true;
                    }
                }else{
                    if(hand.isRight()){
                        msg.append("\n   Task Hand (Right) : ");
                        leftHand(hand, msg);
                        isLeft = true;
                    } else if(hand.isLeft()){
                        msg.append("\n   Action Hand(Left): ");
                        rightHand(hand, msg);
                        isRight = true;
                    }
                }

            }
        }else{
            mListener.onHands(false);
        }
        if(!isRight)
            mListener.onMoveRightHand(null);
        if(!isLeft)
            mListener.onMoveLeftHand(null);
        mListener.onUpdateMsg(msg.toString());
    }

    private void rightHand(Hand hand, StringBuffer msg){

        Vector[] rightPositions = extractPositions(hand,msg);
        if (rightPositions==null)
            return;
        float radius = hand.sphereRadius();
        if (radius > MAX_GRAP)
            radius = MAX_GRAP;
        else if (radius < MIN_GRAP)
            radius = MIN_GRAP;
        Vector handNormal = new Vector(hand.palmNormal());
        handNormal.setX(toRadians(handNormal.getX()));
        handNormal.setY(toRadians(handNormal.getY()));
        handNormal.setZ(toRadians(handNormal.getZ()));
        float grasp = (radius - MIN_GRAP) / (MAX_GRAP - MIN_GRAP);

        msg.append(String.format("\n      Select Task: (%.2f, %.2f, %.2f)", rightPositions[1].getX(), rightPositions[1].getY(), rightPositions[1].getZ()));
        msg.append(String.format("\n      Move Task: (%.2f, %.2f, %.2f)", rightPositions[0].getX(), rightPositions[0].getY(), rightPositions[0].getZ()));
        msg.append(String.format("\n      Rotate Task: (%.2f, %.2f, %.2f)", handNormal.getY(), handNormal.getX(), handNormal.getZ()));
        msg.append(String.format("\n      GraspInv Task: (%.2f)", grasp));

        mListener.onMove(rightPositions[0].getX(), rightPositions[0].getY(), rightPositions[0].getZ());
        mListener.onSelect(rightPositions[1].getX(), rightPositions[1].getY(), rightPositions[1].getZ());
        mListener.onRotate(handNormal.getY(), handNormal.getX(), handNormal.getZ());
        mListener.onGrasping(grasp); //values from 0 to 1.
        mListener.onMoveRightHand(rightPositions);
    }

    private void leftHand(Hand hand, StringBuffer msg){

        Vector[] leftPositions = extractPositions(hand, msg);
        if (leftPositions==null)
            return;

        mListener.onMoveLeftHand(leftPositions);

        Vector palm = leftPositions[0];

        double indexLenght=Math.hypot( leftPositions[0].getX()-leftPositions[1].getX() , leftPositions[0].getY()-leftPositions[1].getY() );
        indexLenght = Math.hypot( indexLenght , leftPositions[0].getZ()-leftPositions[1].getZ() );

        double middleLenght=Math.hypot( leftPositions[0].getX()-leftPositions[2].getX() , leftPositions[0].getY()-leftPositions[2].getY() );
        middleLenght = Math.hypot( middleLenght , leftPositions[0].getZ()-leftPositions[2].getZ() );

        double ringLenght=Math.hypot( leftPositions[0].getX()-leftPositions[3].getX() , leftPositions[0].getY()-leftPositions[3].getY() );
        ringLenght = Math.hypot( ringLenght , leftPositions[0].getZ()-leftPositions[3].getZ() );

        double pinkyLenght=Math.hypot(leftPositions[0].getX() - leftPositions[4].getX(), leftPositions[0].getY() - leftPositions[4].getY());
        pinkyLenght = Math.hypot( pinkyLenght , leftPositions[0].getZ()-leftPositions[4].getZ() );

        double thumbLenght=Math.hypot(leftPositions[0].getX() - leftPositions[5].getX(), leftPositions[0].getY() - leftPositions[5].getY());
        thumbLenght = Math.hypot( thumbLenght , leftPositions[0].getZ()-leftPositions[5].getZ() );

        msg.append(String.format("\n      Distances (I,M,R,P,T) => (%.4f, %.4f, %.4f, %.4f, %.4f)", indexLenght, middleLenght, ringLenght, pinkyLenght, thumbLenght));

        boolean isIndex = indexLenght > OPEN_THRESHOLD;
        boolean isMiddle = middleLenght > OPEN_THRESHOLD;
        boolean isRing = ringLenght > OPEN_THRESHOLD;
        boolean isPinky = pinkyLenght > OPEN_THRESHOLD;
        boolean isThumb = thumbLenght > OPEN_THRESHOLD;

        int numOfFingers=0;


        if(isIndex && isMiddle && isRing && isPinky && isThumb){
            msg.append("\n      Task: 5 (open hand, no task)");
            mListener.onTask(5);
            return; //nothing just the hand open...
        }

        if(isIndex)
            numOfFingers++;
        if(isMiddle)
            numOfFingers++;
        if(isRing)
            numOfFingers++;
        if(isPinky)
            numOfFingers++;

        if(!isThumb){
            msg.append("\n      Task: "+numOfFingers);
            mListener.onTask(numOfFingers);
        }else if(numOfFingers>0){
            msg.append("\n      Task: -1 (not valid)");
            mListener.onTask(-1); //not valid gesture
        }else{
            msg.append("\n      Task: 6 (confirm)");
            mListener.onTask(6); //confirm
        }
    }

    private Vector[] extractPositions(Hand hand, StringBuffer msg){
        int numFingers=0;
        Vector[] positions = new Vector[6];

        msg.append("\n      Palm: ");
        positions[0] = applyPositionLimits( new Vector(hand.palmPosition()), msg); //palm -> 0;


        FingerList fingers = hand.fingers();

        for (Finger finger : fingers) {
            numFingers++;
            if (finger.type().equals(Finger.Type.TYPE_INDEX)){
                msg.append("\n         Index:  ");
                positions[1] = applyPositionLimits( new Vector(finger.tipPosition()), msg); //index -> 1;
            } else if(finger.type().equals(Finger.Type.TYPE_MIDDLE)){
                msg.append("\n         Middle: ");
                positions[2] = applyPositionLimits( new Vector(finger.tipPosition()), msg); //middle -> 2;
            } else if(finger.type().equals(Finger.Type.TYPE_RING)){
                msg.append("\n         Ring:   ");
                positions[3] = applyPositionLimits( new Vector(finger.tipPosition()), msg); //ring -> 3;
            } else if(finger.type().equals(Finger.Type.TYPE_PINKY)){
                msg.append("\n         Pinky:  ");
                positions[4] = applyPositionLimits( new Vector(finger.tipPosition()), msg); //pinky -> 4;
            }else if(finger.type().equals(Finger.Type.TYPE_THUMB)){
                msg.append("\n         Thumb:  ");
                positions[5] = applyPositionLimits( new Vector(finger.tipPosition()), msg); //thumb -> 5;
            }
        }

        if(numFingers<5)
            return null;
        return positions;
    }

    private Vector applyPositionLimits(Vector position, StringBuffer msg){
        msg.append(String.format("(%.0f, %.0f, %.0f) -> ", position.getX(), position.getY(), position.getZ()));
        if(position.getX() > MAX_POS_X)
            position.setX(MAX_POS_X);
        else if(position.getX() < MIN_POS_X)
            position.setX(MIN_POS_X);
        if(position.getY() > MAX_POS_Y)
            position.setY(MAX_POS_Y);
        else if(position.getY() < MIN_POS_Y)
            position.setY(MIN_POS_Y);
        if(position.getZ() > MAX_POS_Z)
            position.setZ(MAX_POS_Z);
        else if(position.getZ() < MIN_POS_Z)
            position.setZ(MIN_POS_Z);

        Vector result = new Vector( (1 + position.getX() / MAX_POS_X)/2f, (1 + position.getY() / MAX_POS_Y)/2f, (1 + position.getZ() / MAX_POS_Z)/2); //values from 0 to 1
        msg.append(String.format("(%.4f, %.4f, %.4f) ", result.getX(), result.getY(), result.getZ()));
        return result;
    }

    private float toRadians(float dir){
        return dir*3.1416f/2f;
    }

    public boolean isRightHanded() {
        return rightHanded;
    }

    public void setRightHanded(boolean rightHanded) {
        this.rightHanded = rightHanded;
    }

    public static interface LeapMotionFrameListener {
        public void onHands(boolean hands);
        public void onSelect(float x, float y, float z);
        public void onMove(float x, float y, float z);
        public void onRotate(float x, float y, float z);
        public void onGrasping(float g);

        public void onTask(int task);

        public void onUpdateMsg(String msg);
        public void onMoveLeftHand(Vector[] positions);
        public void onMoveRightHand(Vector[] positions);
    }
}