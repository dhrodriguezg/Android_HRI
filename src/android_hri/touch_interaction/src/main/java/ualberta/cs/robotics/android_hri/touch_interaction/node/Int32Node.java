package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Int32;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class Int32Node implements NodeMain {

    private String nodeName = "customInt";
    private Publisher<Int32> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private int publish_int=0;
    private int subcribe_int=0;

    private String subscribeTopic;
    private String publishTopic;

    private boolean alwaysPublish;
    private int maxPublishing = 0;
    private long publishFreq = 10;

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
            publisher = connectedNode.newPublisher(publishTopic, Int32._TYPE);
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
                Thread.sleep(publishFreq);
            }
        };
        connectedNode.executeCancellableLoop(aLoop);
    }

    private void publish(){
        Int32 int32 = publisher.newMessage();
        int32.setData(publish_int);
        publisher.publish(int32);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Int32> subscriber = connectedNode.newSubscriber(subscribeTopic, Int32._TYPE);

        subscriber.addMessageListener(new MessageListener<Int32>() {
            @Override
            public void onNewMessage(Int32 int32) {
                subcribe_int =int32.getData();
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

    public int getPublish_int() {
        return publish_int;
    }

    public void setPublish_int(int publish_int) {
        this.publish_int = publish_int;
    }

    public int getSubcribe_int() {
        return subcribe_int;
    }

    public void setSubcribe_int(int subcribe_int) {
        this.subcribe_int = subcribe_int;
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

    public long getPublishFreq() {
        return publishFreq;
    }

    public void setPublishFreq(long publishFreq) {
        this.publishFreq = publishFreq;
    }
}
