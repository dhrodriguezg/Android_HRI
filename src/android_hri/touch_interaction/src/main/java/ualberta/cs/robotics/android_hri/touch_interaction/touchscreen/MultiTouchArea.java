package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen;

import android.app.Activity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.ImageView;

import ualberta.cs.robotics.android_hri.touch_interaction.utils.DoubleDragGestureDetector;
import ualberta.cs.robotics.android_hri.touch_interaction.utils.RotationGestureDetector;

/**
 * Created by DarkNeoBahamut on 23/07/2015.
 */
public class MultiTouchArea extends TouchArea implements RotationGestureDetector.OnRotationGestureListener, DoubleDragGestureDetector.OnDragGestureListener {

    private RotationGestureDetector mRotationDetector;
    private DoubleDragGestureDetector mDragDetector;
    protected float angle;
    protected float dragX;
    protected float dragY;
    protected boolean multiChanged = false;

    public MultiTouchArea(Activity activity, ImageView view) {
        super(activity, view);
    }

    @Override
    protected void setupListener() {
        mRotationDetector = new RotationGestureDetector(this);
        mDragDetector = new DoubleDragGestureDetector(this);
        view.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent event) {
                mRotationDetector.onTouchEvent(event);
                mDragDetector.onTouchEvent(view, event);
                if (event.getPointerCount() < 2) {
                    mSingleDragGestureDetector.onTouchEvent(view, event);
                    mDoubleTapDetector.onTouchEvent(view, event);
                    mLongClickGestureDetector.onTouchEvent(view, event);
                }
                return true;
            }
        });
    }

    @Override
    public void OnRotation(RotationGestureDetector rotationDetector) {
        angle = rotationDetector.getAngle();
        Log.d("Log: Rotation", "Angle:" + Float.toString(angle));
    }

    @Override
    public void OnDoubleDrag(DoubleDragGestureDetector doubleDragDetector) {
        dragX = doubleDragDetector.getX();
        dragY = doubleDragDetector.getY();
        Log.d("Log: DoubleDrag", "X:" + dragX + " Y:" + dragY);
    }
}
