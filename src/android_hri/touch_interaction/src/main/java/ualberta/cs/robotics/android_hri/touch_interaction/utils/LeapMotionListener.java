package ualberta.cs.robotics.android_hri.touch_interaction.utils;
/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

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
        //logic for left hand;

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
            //mListener.onUpdateMsg(msg.toString());
        }else{
            mListener.onHands(false);
        }
    }

    private void rightHand(Hand hand, StringBuffer msg){

        Vector position = new Vector(hand.palmPosition());
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
        mListener.onMove(1 - position.getX() / MAX_POS_X, 1 - position.getY() / MAX_POS_Y, 1 + position.getZ() / MAX_POS_Z); //values from 0 to 2...

        float radius = hand.sphereRadius();
        if (radius > MAX_GRAP)
            radius = MAX_GRAP;
        else if (radius < MIN_GRAP)
            radius = MIN_GRAP;
        mListener.onGrasping((radius - MIN_GRAP) / (MAX_GRAP - MIN_GRAP)); //values from 0 to 1.

        Vector handNormal = hand.palmNormal();
        mListener.onRotate( toRadians(handNormal.getY()), toRadians(handNormal.getX()), toRadians(handNormal.getZ()));

        for (Finger finger : hand.fingers()){
            if (finger.type().equals(Finger.Type.TYPE_INDEX)){
                Vector indexTip = finger.tipPosition();
                if(indexTip.getX() > MAX_POS_X)
                    indexTip.setX(MAX_POS_X);
                else if(indexTip.getX() < MIN_POS_X)
                    indexTip.setX(MIN_POS_X);

                if(indexTip.getY() > MAX_POS_Y)
                    indexTip.setY(MAX_POS_Y);
                else if(indexTip.getY() < MIN_POS_Y)
                    indexTip.setY(MIN_POS_Y);

                if(indexTip.getZ() > MAX_POS_Z)
                    indexTip.setZ(MAX_POS_Z);
                else if(indexTip.getZ() < MIN_POS_Z)
                    indexTip.setZ(MIN_POS_Z);

                mListener.onSelect(1 - indexTip.getX() / MAX_POS_X, 1 - indexTip.getY() / MAX_POS_Y, 1 + indexTip.getZ() / MAX_POS_Z); //values from 0 to 2...
            }
        }
    }

    private void leftHand(Hand hand, StringBuffer msg){
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
        public void onUpdateMsg(String msg);
    }
}