package ualberta.cs.robotics.android_hri.touch_interaction.node;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Bool;

/**
 * Created by dhrodriguezg on 7/29/15.
 */
public class BooleanNode implements NodeMain {

    private String nodeName = "customButton";
    private Publisher<Bool> publisher = null;
    private int counter = 0;

    private boolean isSubscriber;
    private boolean isPublisher;

    private boolean publish_bool;
    private boolean subcribe_bool;

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
            publisher = connectedNode.newPublisher(publishTopic, Bool._TYPE);
            publish(connectedNode);
        }
        if(isSubscriber){
            subscribe(connectedNode);
        }
    }


    private void publish(ConnectedNode connectedNode){
        final CancellableLoop aLoop = new CancellableLoop() {
            @Override protected void loop() throws InterruptedException {
                if(alwaysPublish){
                    publish();
                    if(counter==0 && maxPublishing!=0){
                        publish_bool=false;
                    }
                }else{
                    if(counter>0)
                        publish();
                }

                counter--;
                if(counter<0)
                    counter=0;
                Thread.sleep(publishFreq);
            }
        };
        connectedNode.executeCancellableLoop(aLoop);
    }

    private void publish(){
        Bool bool = publisher.newMessage();
        bool.setData(publish_bool);
        publisher.publish(bool);
        hasPublishedMsg=true;
    }

    public void publishNow(){
        counter=maxPublishing;
    }

    private void subscribe(ConnectedNode connectedNode) {
        Subscriber<Bool> subscriber = connectedNode.newSubscriber(subscribeTopic, Bool._TYPE);

        subscriber.addMessageListener(new MessageListener<Bool>() {
            @Override
            public void onNewMessage(Bool bool) {
                subcribe_bool = bool.getData();
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

    public boolean isPublish_bool() {
        return publish_bool;
    }

    public void setPublish_bool(boolean publish_bool) {
        this.publish_bool = publish_bool;
    }

    public boolean isSubcribe_bool() {
        return subcribe_bool;
    }

    public void setSubcribe_bool(boolean subcribe_bool) {
        this.subcribe_bool = subcribe_bool;
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
