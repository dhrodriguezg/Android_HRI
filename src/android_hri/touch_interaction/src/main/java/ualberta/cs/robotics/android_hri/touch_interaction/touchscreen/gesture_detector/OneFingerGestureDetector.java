package ualberta.cs.robotics.android_hri.touch_interaction.touchscreen.gesture_detector;

import android.app.Activity;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Diego Rodriguez on 25/07/2015.
 */
public class OneFingerGestureDetector extends GestureDetector.SimpleOnGestureListener {

    private static final String TAG = "OneFingerGesture";

    private OnOneFingerGestureListener mListener;
    private GestureDetector mGestureDetector;
    private Activity mActivity;
    private View mView;
    private boolean detectingGesture =false;

    //onSingleTapUp -> one single update
    private float stX;
    private float stY;
    private float normalizedSTX;
    private float normalizedSTY;
    private boolean enableSingleTapUp=false;

    //onDoubleTap -> one single update
    private float dtX;
    private float dtY;
    private float normalizedDTX;
    private float normalizedDTY;
    private boolean enableDoubleTap=false;

    //onScroll -> constant updating, relative distance
    private float sX;
    private float sY;
    private float sdX;
    private float sdY;
    private float dX, dY;
    private float normalizedDX;
    private float normalizedDY;
    private boolean enableScroll=false;

    //onFling -> one single update, absolute velocity
    private float fX;
    private float fY;
    private float fvX;
    private float fvY;
    private boolean enableFling=false;

    //onLongPress -> one single update.
    private float lpX;
    private float lpY;
    private float normalizedLPX;
    private float normalizedLPY;
    private boolean enableLongPress=false;

    private boolean  detectingMultiGesture=false;

    public OneFingerGestureDetector(Activity activity, OnOneFingerGestureListener listener){
        mGestureDetector = new GestureDetector(activity, this);
        mActivity = activity;
        mListener = listener;
    }

    /** Low-level events **/

    public boolean onTouchEvent(View view, MotionEvent event){
        if (event.getPointerCount() < 2) {
            detectingMultiGesture=false;
            mView = view;
            checkGestureState(event);
            return mGestureDetector.onTouchEvent(event);
        }else{
            detectingMultiGesture=true;
        }
        return true;
    }

    private void checkGestureState(MotionEvent event){
        if(event.getAction()==MotionEvent.ACTION_DOWN){
            detectingGesture = true;
            mListener.onOneFingerGestureState(detectingGesture);
            Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
        }else if(event.getAction()==MotionEvent.ACTION_UP){
            detectingGesture = false;
            mListener.onOneFingerGestureState(detectingGesture);
            Log.d(TAG, String.format("Detecting [ %b ]", detectingGesture));
        }
    }

    @Override
    public boolean onDown(MotionEvent event){
        return super.onDown(event);
    }

    @Override
    public void onShowPress(MotionEvent event){
        super.onShowPress(event);
    }


    /** Gesture events **/
    @Override
    public boolean onSingleTapUp(MotionEvent event) {
        if(!enableSingleTapUp)
            return true;
        stX =event.getX(); stY =event.getY();
        float[] normalizedXY = normalizedValues(event,mView);
        normalizedSTX = normalizedXY[0];
        normalizedSTY = normalizedXY[1];
        mListener.onSingleTap(stX, stY, normalizedSTX, normalizedSTY);
        Log.d(TAG, String.format("SingleTap [ %.1f , %.1f , %.4f , %.4f ]", stX, stY, normalizedSTX, normalizedSTY));
        return true;
    }

    @Override
    public boolean onDoubleTap(MotionEvent event) {
        if(!enableDoubleTap)
            return true;
        dtX =event.getX(); dtY =event.getY();
        float[] normalizedXY = normalizedValues(event,mView);
        normalizedDTX = normalizedXY[0];
        normalizedDTY = normalizedXY[1];
        mListener.onDoubleTap(dtX, dtY, normalizedDTX, normalizedDTY);
        Log.d(TAG, String.format("DoubleTap [ %.1f , %.1f , %.4f , %.4f ]", dtX, dtY, normalizedDTX, normalizedDTY));
        return true;
    }

    @Override
    public boolean onScroll(MotionEvent initial_event, MotionEvent current_event, float distanceX, float distanceY) {
        if(!enableScroll)
            return true;
        sX =initial_event.getX(); sY =initial_event.getY();
        sdX = distanceX; sdY=distanceY;

        dX=current_event.getX(); dY=current_event.getY();
        float[] normalizedXY = normalizedValues(current_event,mView);
        normalizedDX = normalizedXY[0];
        normalizedDY = normalizedXY[1];

        mListener.onScrollDrag(sX, sY, sdX, sdY, dX, dY, normalizedDX, normalizedDY);
        Log.d(TAG, String.format("Scroll [ %.1f , %.1f , %.4f , %.4f / %.1f , %.1f , %.4f , %.4f ]", sX, sY, sdX, sdY, dX, dY, normalizedDX, normalizedDY));
        return true;
    }

    @Override
    public boolean onFling(MotionEvent initial_event, MotionEvent current_event, float velocityX, float velocityY) {
        if(!enableFling)
            return true;
        fX = initial_event.getX();
        fY = initial_event.getY();
        fvX = velocityX; fvY = velocityY;
        mListener.onFling(fX, fY, fvX, fvY);
        Log.d(TAG, String.format("Fling [ %.4f , %.4f , %.4f , %.4f ]", fX, fY, fvX, fvY));
        return true;
    }

    @Override
    public void onLongPress(MotionEvent event) {
        if(!enableLongPress)
            return;
        if(detectingMultiGesture)
            return;

        lpX =event.getX(); lpY =event.getY();
        float[] normalizedXY = normalizedValues(event,mView);
        normalizedLPX = normalizedXY[0];
        normalizedLPY = normalizedXY[1];
        mListener.onLongPress(lpX, lpY, normalizedLPX,normalizedLPY);
        Log.d(TAG, String.format("LongPress [ %.1f , %.1f , %.4f , %.4f ]", lpX, lpY, normalizedLPX, normalizedLPY));
    }

    /** Gesture interfaces **/

    public static interface OnOneFingerGestureListener {
        public void onSingleTap(float stX,float stY, float normalizedSTX, float normalizedSTY);
        public void onDoubleTap(float dtX, float dtY, float normalizedDTX, float normalizedDTY);
        public void onScrollDrag(float sX, float sY, float sdX, float sdY, float dX, float dY, float normalizedDX, float normalizedDY);
        public void onFling(float fX, float fY, float fvX, float fvY);
        public void onLongPress(float lpX,float lpY,float normalizedLPX,float normalizedLPY);
        public void onOneFingerGestureState(boolean detectingGesture);
    }

    /** Normalized values **/

    private float[] normalizedValues(MotionEvent event, View view){
        float[] touch = new float[]{event.getX(), event.getY()}; //relative to the view. Right,Down increase
        float[] viewCenter = new float[]{view.getWidth() / 2, view.getHeight() / 2};
        float[] touchVector = new float[] {touch[0] - viewCenter[0], touch[1] - viewCenter[1]};
        touchVector[0]/=  viewCenter[0];
        touchVector[1]/= -viewCenter[1];
        if(touchVector[0] > 1 || touchVector[0] < -1 || touchVector[1] > 1 || touchVector[1] < -1){
            touchVector[0] = 0;
            touchVector[1] = 0;
        }
        return touchVector;
    }

    /** Getters and Setters **/
    public boolean isDetectingGesture() {
        return detectingGesture;
    }

    public void setDetectingGesture(boolean detectingGesture) {
        this.detectingGesture = detectingGesture;
    }

    public float getStX() {
        return stX;
    }

    public void setStX(float stX) {
        this.stX = stX;
    }

    public float getStY() {
        return stY;
    }

    public void setStY(float stY) {
        this.stY = stY;
    }

    public float getNormalizedSTX() {
        return normalizedSTX;
    }

    public void setNormalizedSTX(float normalizedSTX) {
        this.normalizedSTX = normalizedSTX;
    }

    public float getNormalizedSTY() {
        return normalizedSTY;
    }

    public void setNormalizedSTY(float normalizedSTY) {
        this.normalizedSTY = normalizedSTY;
    }

    public float getDtX() {
        return dtX;
    }

    public void setDtX(float dtX) {
        this.dtX = dtX;
    }

    public float getDtY() {
        return dtY;
    }

    public void setDtY(float dtY) {
        this.dtY = dtY;
    }

    public float getNormalizedDTX() {
        return normalizedDTX;
    }

    public void setNormalizedDTX(float normalizedDTX) {
        this.normalizedDTX = normalizedDTX;
    }

    public float getNormalizedDTY() {
        return normalizedDTY;
    }

    public void setNormalizedDTY(float normalizedDTY) {
        this.normalizedDTY = normalizedDTY;
    }

    public float getsX() {
        return sX;
    }

    public void setsX(float sX) {
        this.sX = sX;
    }

    public float getsY() {
        return sY;
    }

    public void setsY(float sY) {
        this.sY = sY;
    }

    public float getSdX() {
        return sdX;
    }

    public void setSdX(float sdX) {
        this.sdX = sdX;
    }

    public float getSdY() {
        return sdY;
    }

    public void setSdY(float sdY) {
        this.sdY = sdY;
    }

    public float getdX() {
        return dX;
    }

    public void setdX(float dX) {
        this.dX = dX;
    }

    public float getdY() {
        return dY;
    }

    public void setdY(float dY) {
        this.dY = dY;
    }

    public float getNormalizedDX() {
        return normalizedDX;
    }

    public void setNormalizedDX(float normalizedDX) {
        this.normalizedDX = normalizedDX;
    }

    public float getNormalizedDY() {
        return normalizedDY;
    }

    public void setNormalizedDY(float normalizedDY) {
        this.normalizedDY = normalizedDY;
    }

    public float getfX() {
        return fX;
    }

    public void setfX(float fX) {
        this.fX = fX;
    }

    public float getfY() {
        return fY;
    }

    public void setfY(float fY) {
        this.fY = fY;
    }

    public float getFvX() {
        return fvX;
    }

    public void setFvX(float fvX) {
        this.fvX = fvX;
    }

    public float getFvY() {
        return fvY;
    }

    public void setFvY(float fvY) {
        this.fvY = fvY;
    }

    public float getLpX() {
        return lpX;
    }

    public void setLpX(float lpX) {
        this.lpX = lpX;
    }

    public float getLpY() {
        return lpY;
    }

    public void setLpY(float lpY) {
        this.lpY = lpY;
    }

    public float getNormalizedLPX() {
        return normalizedLPX;
    }

    public void setNormalizedLPX(float normalizedLPX) {
        this.normalizedLPX = normalizedLPX;
    }

    public float getNormalizedLPY() {
        return normalizedLPY;
    }

    public void setNormalizedLPY(float normalizedLPY) {
        this.normalizedLPY = normalizedLPY;
    }

    public void disableSingleTapUp(){
        enableSingleTapUp=false;
    }

    public void disableDoubleTap(){
        enableDoubleTap=false;
    }

    public void disableScroll(){
        enableScroll=false;
    }

    public void disableFling(){
        enableFling=false;
    }

    public void disableLongPress(){
        enableLongPress=false;
    }

    public void enableSingleTapUp(){
        enableSingleTapUp=true;
    }

    public void enableDoubleTap(){
        enableDoubleTap=true;
    }

    public void enableScroll(){
        enableScroll=true;
    }

    public void enableFling(){
        enableFling=true;
    }

    public void enableLongPress(){
        enableLongPress=true;
    }











}
