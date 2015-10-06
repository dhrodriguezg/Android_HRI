package ualberta.cs.robotics.android_hri.touch_interaction.service;

import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;

import rosjava_test_msgs.AddTwoIntsRequest;
import rosjava_test_msgs.AddTwoIntsResponse;
import std_msgs.Bool;

/**
 * Created by dhrodriguezg on 9/28/15.
 */
public abstract class AbstractService {

    protected boolean isServer;
    protected boolean isClient;

    protected String serverTopic;
    protected String clientTopic;

    protected boolean hasServerMsg =false;
    protected boolean hasClientMsg =false;

    public void serverOf(String topic) {
        isServer = true;
        serverTopic = topic;
    }

    public void clientOf(String topic) {
        isClient = true;
        clientTopic = topic;
    }

    public void onStart(ConnectedNode connectedNode) {
        if(isServer)
            setupServer(connectedNode);
        if(isClient)
            setupClient(connectedNode);
    }

    public boolean hasReceivedMsg() {
        return hasServerMsg;
    }

    public void setHasServerMsg(boolean hasServerMsg) {
        this.hasServerMsg = hasServerMsg;
    }

    public boolean hasPublishedMsg() {
        return hasClientMsg;
    }

    public void setHasClientMsg(boolean hasClientMsg) {
        this.hasClientMsg = hasClientMsg;
    }

    protected abstract void setupClient(final ConnectedNode connectedNode);
    protected abstract void publish();
    protected abstract void setupServer(final ConnectedNode connectedNode);

}
