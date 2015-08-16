package ualberta.cs.robotics.android_hri.touch_interaction;
/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

import android.util.Log;

import com.leapmotion.leap.CircleGesture;
import com.leapmotion.leap.Controller;
import com.leapmotion.leap.Finger;
import com.leapmotion.leap.FingerList;
import com.leapmotion.leap.Frame;
import com.leapmotion.leap.Gesture;
import com.leapmotion.leap.Gesture.State;
import com.leapmotion.leap.GestureList;
import com.leapmotion.leap.Hand;
import com.leapmotion.leap.KeyTapGesture;
import com.leapmotion.leap.Listener;
import com.leapmotion.leap.ScreenTapGesture;
import com.leapmotion.leap.SwipeGesture;
import com.leapmotion.leap.Vector;


public class SampleListener extends Listener {

    private static final String TAG = "SampleListener";
    private LeapActivity mActivity;
    private long mPrevTimeStamp;
    private int mDuration = 30;
    private int mCount = 0;
    private float mFps = 0;

    public SampleListener(LeapActivity activity) {
       mActivity = activity;
    }

    public void onInit(Controller controller) {
        Log.d(TAG, "Initialized");
    }

    public void onConnect(Controller controller) {
        mActivity.printLog("Connected");
        controller.enableGesture(Gesture.Type.TYPE_SWIPE);
        controller.enableGesture(Gesture.Type.TYPE_CIRCLE);
        controller.enableGesture(Gesture.Type.TYPE_SCREEN_TAP);
        controller.enableGesture(Gesture.Type.TYPE_KEY_TAP);
    }

    public void onDisconnect(Controller controller) {
        //Note: not dispatched when running in a debugger.
        mActivity.printLog("Disconnected");
    }

    public void onExit(Controller controller) {
        mActivity.printLog("Exited");
    }

    public void onFrame(Controller controller) {
        // Get the most recent frame and report some basic information
        Frame frame = controller.frame();
        if (++mCount == mDuration) {
            long spent_time = (frame.timestamp() - mPrevTimeStamp) / mCount;
            mFps = 1000000.0f / spent_time;
            mActivity.printLog("data fps: " + mFps);
            mActivity.addLog("hands: " + frame.hands().count()
                    + ", fingers: " + frame.fingers().count());
            mCount = 0;
            mPrevTimeStamp = frame.timestamp();
        }
        if (!frame.hands().isEmpty()) {
            // Get the first hand
            Hand hand = frame.hands().get(0);

            // Check if the hand has any fingers
            FingerList fingers = hand.fingers();
            if (!fingers.isEmpty()) {
                // Calculate the hand's average finger tip position
                Vector avgPos = Vector.zero();
                for (Finger finger : fingers) {
                    avgPos = avgPos.plus(finger.tipPosition());
                }
                avgPos = avgPos.divide(fingers.count());
            }

            // Get the hand's normal vector and direction
            Vector normal = hand.palmNormal();
            Vector direction = hand.direction();
        }

        GestureList gestures = frame.gestures();
        for (int i = 0; i < gestures.count(); i++) {
            Gesture gesture = gestures.get(i);

            switch (gesture.type()) {
                case TYPE_CIRCLE:
                    CircleGesture circle = new CircleGesture(gesture);

                    // Calculate clock direction using the angle between circle normal and pointable
                    String clockwiseness;
                    if (circle.pointable().direction().angleTo(circle.normal()) <= Math.PI/4) {
                        // Clockwise if angle is less than 90 degrees
                        clockwiseness = "clockwise";
                    } else {
                        clockwiseness = "counterclockwise";
                    }

                    // Calculate angle swept since last frame
                    double sweptAngle = 0;
                    if (circle.state() != State.STATE_START) {
                        CircleGesture previousUpdate = new CircleGesture(controller.frame(1).gesture(circle.id()));
                        sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * Math.PI;
                    }
                    break;
                case TYPE_SWIPE:
                    SwipeGesture swipe = new SwipeGesture(gesture);
                    break;
                case TYPE_SCREEN_TAP:
                    ScreenTapGesture screenTap = new ScreenTapGesture(gesture);
                    break;
                case TYPE_KEY_TAP:
                    KeyTapGesture keyTap = new KeyTapGesture(gesture);
                    break;
                default:
                    Log.d(TAG, "Unknown gesture type.");
                    break;
            }
        }
    }
}