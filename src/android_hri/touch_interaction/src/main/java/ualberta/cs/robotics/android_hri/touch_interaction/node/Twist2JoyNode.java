package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Twist;
import sensor_msgs.Joy;
import geometry_msgs.Point;
import std_msgs.Int32;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class Twist2JoyNode implements NodeMain {

    private java.lang.String nodeName = "customJoy";
    private Publisher<sensor_msgs.Joy> publisher = null;
    private float[] axes = new float[]{0, 0, 0, 0, 0, 0};
    private int[] button = new int[]{0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0};
    private int counter = 100;
    private int graspValue = -1;
    private boolean started=false;

    @Override
    public void onStart(ConnectedNode node) {
        publisher = node.newPublisher("/joy", Joy._TYPE);  //"/android/joynode/joy"
        getTwistPos(node);
        getTwistRot(node);
        setJoy(node);
    }

    private void getTwistPos(ConnectedNode connectedNode) {
        Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("/android/joystickPos/cmd_vel", geometry_msgs.Twist._TYPE);

        subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(geometry_msgs.Twist message) {
                sensor_msgs.Joy joy = publisher.newMessage();
                axes[0] = (float) message.getLinear().getY();
                axes[1] = (float) message.getLinear().getX();
            }
        });
    }

    private void getTwistRot(ConnectedNode connectedNode) {
        Subscriber<geometry_msgs.Twist> subscriber = connectedNode.newSubscriber("/android/joystickRot/cmd_vel", geometry_msgs.Twist._TYPE);

        subscriber.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
            @Override
            public void onNewMessage(geometry_msgs.Twist message) {
                sensor_msgs.Joy joy = publisher.newMessage();
                axes[2]=(float)message.getLinear().getY();
                axes[3]=(float)message.getLinear().getX();
            }
        });
    }

    private void setJoy(ConnectedNode node) {

        Point p=null;
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                if(started){
                    counter--;
                }
                sensor_msgs.Joy joy = publisher.newMessage();
                joy.setAxes(axes);
                joy.setButtons(button);
                if(counter<0){
                    button[12]=0;
                    button[14]=0;
                }

                publisher.publish(joy);
                Thread.sleep(10);
            }
        };
        node.executeCancellableLoop(aLoop);
    }

    public void setGraspValue(int value){
        if(graspValue!=value){
            graspValue=value;
            counter=100;
            started=true;
        }
        if(graspValue==0){
            button[12]=1;
            button[14]=0;
        }else{
            button[12]=0;
            button[14]=1;
        }
    }

    public void setButtonValue(int button, int value) {
        this.button[button]=value;
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
}
