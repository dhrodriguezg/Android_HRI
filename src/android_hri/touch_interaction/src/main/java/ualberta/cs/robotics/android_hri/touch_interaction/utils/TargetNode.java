package ualberta.cs.robotics.android_hri.touch_interaction.utils;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class TargetNode implements NodeMain {

    private String nodeName = "customTarget";
    private Publisher<Point> publisher = null;
    private float x=0;
    private float y=0;
    private float z=0;
    private int timer = 100;
    private boolean isValueSet = false;

    @Override
    public void onStart(ConnectedNode node) {
        publisher = node.newPublisher("/android/target_point", Point._TYPE);
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                if (isValueSet)
                    timer--;
                Point point = publisher.newMessage();
                point.setX(x);
                point.setY(y);
                point.setZ(z);
                if(timer>0){
                    publisher.publish(point);
                }
                Thread.sleep(10);
            }
        };
        node.executeCancellableLoop(aLoop);

    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        if( Math.abs(x-this.x) > 2){
            this.x = x;
            timer = 10;
        }
        isValueSet=true;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        if( Math.abs(y-this.y) > 2){
            this.y = y;
            timer = 10;
        }
        isValueSet=true;
    }

    public float getZ() {
        return z;
    }

    public void setZ(float z) {
        this.z = z;
    }
}
