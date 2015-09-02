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
    private static final float MIN_POS_Y = -200;
    private static final float MIN_POS_Z = -200;
    private static final float MAX_POS_X =  200;
    private static final float MAX_POS_Y =  200;
    private static final float MAX_POS_Z =  200;
    private static final float MIN_GRAP =  30;
    private static final float MAX_GRAP =  60;
    private static final float OPEN_THRESHOLD =  0.1875f;

    private LeapMotionFrameListener mListener;
    private Activity mActivity;
    private long mPrevTimeStamp;
    private int mDuration = 30;
    private int mCount = 0;
    private float mFps = 0;

    public LeapMotionListener(Activity activity, LeapMotionFrameListener listener) {
        mActivity = activity;
        mListener = listener;
    }

    public void onInit(Controller controller) {
        Log.d(TAG, "Initialized");
    }

    public void onConnect(Controller controller) {
        mListener.onUpdateMsg("Connected");
    }

    public void onDisconnect(Controller controller) {
        //Note: not dispatched when running in a debugger.
        mListener.onUpdateMsg("Disconnected");
    }

    public void onExit(Controller controller) {
        mListener.onUpdateMsg("Exited");
    }

    public void onFrame(Controller controller) {
        StringBuffer msg = new StringBuffer();
        // Get the most recent frame and report some basic information
        Frame frame = controller.frame();
        if (++mCount == mDuration) {
            long spent_time = (frame.timestamp() - mPrevTimeStamp) / mCount;
            mFps = 1000000.0f / spent_time;
            msg.append("\nfps: " + mFps);
            msg.append("\nhands: " + frame.hands().count() + ", fingers: " + frame.fingers().count());
            mCount = 0;
            mPrevTimeStamp = frame.timestamp();
        }

        if (!frame.hands().isEmpty()) {
            mListener.onHands(true);
            for (Hand hand : frame.hands() ){
                if(!hand.isValid())
                    return;
                if(hand.isRight())
                    rightHand(hand, msg);
                else if(hand.isLeft())
                    leftHand(hand, msg);
            }
            mListener.onUpdateMsg(msg.toString());
        }else{
            mListener.onHands(false);
        }
    }

    private void rightHand(Hand hand, StringBuffer msg){

        Vector[] rightPositions = extractPositions(hand,msg);
        if (rightPositions==null)
            return;

        mListener.onMove(rightPositions[0].getX(), rightPositions[0].getY(), rightPositions[0].getZ());
        mListener.onSelect(rightPositions[1].getX(), rightPositions[1].getY(), rightPositions[1].getZ());

        float radius = hand.sphereRadius();
        if (radius > MAX_GRAP)
            radius = MAX_GRAP;
        else if (radius < MIN_GRAP)
            radius = MIN_GRAP;
        mListener.onGrasping((radius - MIN_GRAP) / (MAX_GRAP - MIN_GRAP)); //values from 0 to 1.

        Vector handNormal = hand.palmNormal();
        mListener.onRotate( toRadians(handNormal.getY()), toRadians(handNormal.getX()), toRadians(handNormal.getZ()));

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

        boolean isIndex = indexLenght > OPEN_THRESHOLD;
        boolean isMiddle = middleLenght > OPEN_THRESHOLD;
        boolean isRing = ringLenght > OPEN_THRESHOLD;
        boolean isPinky = pinkyLenght > OPEN_THRESHOLD;
        boolean isThumb = thumbLenght > OPEN_THRESHOLD;

        if(isIndex && isMiddle && isRing && isPinky && isThumb){
            return; //nothing just the hand open...
        }

        if(isThumb){
            mListener.onTask(0);//thats to confirm.
            return;
        }

        if(isIndex && isMiddle && isRing && isPinky){
            mListener.onTask(4);
            return;
        }

        if(isIndex && isMiddle && isRing){
            mListener.onTask(3);
            return;
        }

        if(isIndex && isMiddle){
            mListener.onTask(2);
            return;
        }

        if(isIndex){
            mListener.onTask(1);
            return;
        }

        mListener.onTask(5);

        /*
        msg.append(String.format("\n    LeftHand Pos: (%.2f, %.2f, %.2f) ", hand.palmPosition().getX(), hand.palmPosition().getY(), hand.palmPosition().getZ()));
        msg.append(String.format("\n             Dir: (%.2f, %.2f, %.2f) ", hand.palmNormal().getX(), hand.palmNormal().getY(), hand.palmNormal().getZ()));

        Finger thumb, index, middle, ring, pinky;
        FingerList fingers = hand.fingers();
        if (!fingers.isEmpty()) {
            // Calculate the hand's average finger tip position
            Vector avgPos = Vector.zero();
            for (Finger finger : fingers) {
                avgPos = avgPos.plus(finger.tipPosition());
                msg.append(String.format("\n        finger position: %s (%.2f, %.2f, %.2f) ", finger.type().name() , finger.tipPosition().getX(), finger.tipPosition().getY(), finger.tipPosition().getZ()));
                msg.append(String.format("\n        finger direction: %s (%.2f, %.2f, %.2f) ", finger.type().name() , finger.direction().getX(), finger.direction().getY(), finger.direction().getZ()));
            }
            avgPos = avgPos.divide(fingers.count());
            msg.append(String.format("\n    AvgFinger (%.2f, %.2f, %.2f) ", avgPos.getX(), avgPos.getY(), avgPos.getZ()));
        }
        */
    }

    private Vector[] extractPositions(Hand hand, StringBuffer msg){
        int numFingers=0;
        Vector[] positions = new Vector[6];
        positions[0] = applyPositionLimits( new Vector(hand.palmPosition()) ); //palm -> 0;

        msg.append(String.format("\n PalmPos: (%.2f, %.2f, %.2f) ", hand.palmPosition().getX(), hand.palmPosition().getY(), hand.palmPosition().getZ()));

        FingerList fingers = hand.fingers();

        for (Finger finger : fingers) {
            numFingers++;
            msg.append(String.format("\n   FingerPos: (%.2f, %.2f, %.2f) ", finger.tipPosition().getX(), finger.tipPosition().getY(), finger.tipPosition().getZ()));
            if (finger.type().equals(Finger.Type.TYPE_INDEX)){
                positions[1] = applyPositionLimits( new Vector(finger.tipPosition()) ); //index -> 1;
            } else if(finger.type().equals(Finger.Type.TYPE_MIDDLE)){
                positions[2] = applyPositionLimits( new Vector(finger.tipPosition()) ); //middle -> 2;
            } else if(finger.type().equals(Finger.Type.TYPE_RING)){
                positions[3] = applyPositionLimits( new Vector(finger.tipPosition()) ); //ring -> 3;
            } else if(finger.type().equals(Finger.Type.TYPE_PINKY)){
                positions[4] = applyPositionLimits( new Vector(finger.tipPosition()) ); //pinky -> 4;
            }else if(finger.type().equals(Finger.Type.TYPE_THUMB)){
                positions[5] = applyPositionLimits( new Vector(finger.tipPosition()) ); //thumb -> 5;
            }
        }

        if(numFingers<5)
            return null;
        return positions;
    }

    private Vector applyPositionLimits(Vector position){
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
        return new Vector( (1 + position.getX() / MAX_POS_X)/2f, (1 - position.getY() / MAX_POS_Y)/2f, (1 + position.getZ() / MAX_POS_Z)/2); //values from 0 to 1
    }

    private float toRadians(float dir){
        return dir*3.1416f/2f;
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