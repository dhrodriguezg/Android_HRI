package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.Joy;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class JoyNode implements NodeMain {

    private String nodeName = "customJoy";
    private Publisher<Joy> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private float[] publish_axes =new float[]{0, 0, 0, 0, 0, 0};
    private int[] publish_button =new int[]{0,0,0,0,0,0,0,0,0,0,1};

    private float[] subcribe_axes =new float[]{0, 0, 0, 0, 0, 0};
    private int[] subcribe_button =new int[]{0,0,0,0,0,0,0,0,0,0,0};

    private String subscribeTopic;
    private String publishTopic;

    private boolean alwaysPublish;
    private int maxPublishing = 0;
    private boolean hasReceivedMsg=false;
    private boolean hasPublishedMsg=false;

    public void subscribeTo(String topic){
        isSubscriber=true;
        subscribeTopic=topic;
    }

    public void publishTo(String topic, boolean alwaysPublishing, int maxPublishings){
        isPublisher=true;
        publishTopic=topic;
        alwaysPublish=alwaysPublishing;
        maxPublishing=maxPublishings;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        if(isPublisher){
            publisher = connectedNode.newPublisher(publishTopic, Joy._TYPE);
            publish(connectedNode);
        }
        if(isSubscriber){
            subscribe(connectedNode);
        }
    }


    private void publish(ConnectedNode connectedNode){
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                if(alwaysPublish)
                    publish();
                else{
                    if(counter>0){
                        publish();
                    }
                    counter--;
                }
                Thread.sleep(10);
            }
        };
        connectedNode.executeCancellableLoop(aLoop);
    }

    private void publish(){
        Joy joy = publisher.newMessage();
        joy.setAxes(publish_axes);
        joy.setButtons(publish_button);
        publisher.publish(joy);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Joy> subscriber = connectedNode.newSubscriber(subscribeTopic, Joy._TYPE);

        subscriber.addMessageListener(new MessageListener<Joy>() {
            @Override
            public void onNewMessage(Joy joy) {
                subcribe_axes=joy.getAxes();
                subcribe_button=joy.getButtons();
                hasReceivedMsg=true;
            }
        });
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

    public float[] getPublish_axes() {
        return publish_axes;
    }

    public void setPublish_axes(float[] publish_axes) {
        this.publish_axes = publish_axes;
    }

    public int[] getPublish_button() {
        return publish_button;
    }

    public void setPublish_button(int[] publish_button) {
        this.publish_button = publish_button;
    }

    public float[] getSubcribe_axes() {
        return subcribe_axes;
    }

    public void setSubcribe_axes(float[] subcribe_axes) {
        this.subcribe_axes = subcribe_axes;
    }

    public int[] getSubcribe_button() {
        return subcribe_button;
    }

    public void setSubcribe_button(int[] subcribe_button) {
        this.subcribe_button = subcribe_button;
    }

    public boolean hasReceivedMsg() {
        return hasReceivedMsg;
    }

    public void setHasReceivedMsg(boolean hasReceivedMsg) {
        this.hasReceivedMsg = hasReceivedMsg;
    }

    public boolean hasPublishedMsg() {
        return hasPublishedMsg;
    }

    public void setHasPublishedMsg(boolean hasPublishedMsg) {
        this.hasPublishedMsg = hasPublishedMsg;
    }
}
