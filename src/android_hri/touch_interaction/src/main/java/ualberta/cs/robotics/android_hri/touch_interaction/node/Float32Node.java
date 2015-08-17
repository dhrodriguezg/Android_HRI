package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Float32;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class Float32Node implements NodeMain {

    private String nodeName = "customFloat";
    private Publisher<Float32> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private float publish_float=0;
    private float subcribe_float=0;

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
            publisher = connectedNode.newPublisher(publishTopic, Float32._TYPE);
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
        Float32 float32 = publisher.newMessage();
        float32.setData(publish_float);
        publisher.publish(float32);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Float32> subscriber = connectedNode.newSubscriber(subscribeTopic, Float32._TYPE);

        subscriber.addMessageListener(new MessageListener<Float32>() {
            @Override
            public void onNewMessage(Float32 float32) {
                subcribe_float=float32.getData();
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

    public float getPublish_float() {
        return publish_float;
    }

    public void setPublish_float(float publish_float) {
        this.publish_float = publish_float;
    }

    public float getSubcribe_float() {
        return subcribe_float;
    }

    public void setSubcribe_float(float subcribe_float) {
        this.subcribe_float = subcribe_float;
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
