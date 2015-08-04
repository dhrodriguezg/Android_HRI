package ualberta.cs.robotics.android_hri.touch_interaction.comm;


import android.util.Log;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;

import ualberta.cs.robotics.android_hri.touch_interaction.ControllerActivity;

public class ClientTCP {
	
	private static final String TAG = "ClientTCP";
	private static final int GREETING_PORT = 7777;
	private static final int STREAMING_PORT = 8888;
	private static final int CONTROLLER_PORT = 9999;
	
	private boolean isDirectWIFI = false;
	private String serverAddress=null;
	
	private boolean isEV3Camera = false;
	
	private Socket streamingSocket = null;
	private DataOutputStream streamingOutput = null;
	private DataInputStream streamingInput = null;
	private boolean isTransferingStreaming = false;
	
	private Socket controllerSocket = null;
	private DataOutputStream controllerOutput = null;
	private DataInputStream controllerInput = null;
	private boolean isTransferingController = false;
	
	private byte[] picture = null;
	private byte[] buffPicture = null;
	private String[] resolutions = null;
	private ControllerActivity activity = null;
	
	public ClientTCP(ControllerActivity act, String host, boolean isP2PiP){
		activity = act;
		serverAddress = host;
		isDirectWIFI = isP2PiP;
	}
	
	public ClientTCP(String host, boolean isP2PiP){
		serverAddress = host;
		isDirectWIFI = isP2PiP;
	}
	
	public boolean greetServer(){
		isEV3Camera = false;
		try {
			Socket serverGreeting;
			if(isDirectWIFI){
				serverGreeting = new Socket();
				serverGreeting.bind(null);
				serverGreeting.connect((new InetSocketAddress(serverAddress, GREETING_PORT)), 500);
			}else{
				serverGreeting = new Socket(serverAddress, GREETING_PORT);
			}
			
			DataOutputStream dataOutput = new DataOutputStream(serverGreeting.getOutputStream());
			DataInputStream dataInput = new DataInputStream(serverGreeting.getInputStream());
			
			dataOutput.writeUTF("Are you EV3 Camera?");
			isEV3Camera = dataInput.readBoolean();
			
    		dataOutput.close();
    		dataInput.close();
    		serverGreeting.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return isEV3Camera;
	}
	
	private void connect2Steaming(){
		Thread thread = new Thread() {
            public void run() {
            	try {
        			streamingSocket = new Socket(serverAddress, STREAMING_PORT);
        			streamingInput = new DataInputStream(streamingSocket.getInputStream());
        			streamingOutput = new DataOutputStream(streamingSocket.getOutputStream());
        			streamingSocket.setKeepAlive(true);
        		} catch (IOException e) {
        			e.printStackTrace();
        		}
            }
        };
        thread.start();
	}
	
	private void connect2Controller(){
		Thread thread = new Thread() {
            public void run() {
            	try {
        			controllerSocket = new Socket(serverAddress, CONTROLLER_PORT);
        			controllerInput = new DataInputStream(controllerSocket.getInputStream());
        			controllerOutput = new DataOutputStream(controllerSocket.getOutputStream());
        			controllerSocket.setKeepAlive(true);
        		} catch (IOException e) {
        			e.printStackTrace();
        		}
            }
        };
        thread.start();
	}
	
	
	public boolean updateStream(){
		
		if(isTransferingStreaming)
			return false; //Is still transfering data, skip the next frame.
		
		boolean requestCompleted = false;
		boolean reconnect = false;
		isTransferingStreaming = true;
		int requestNumber = 0;
		
		if(streamingSocket==null)
			reconnect=true;
		
		while (!requestCompleted && requestNumber++ < 100){ //100 tries
			//Log.i(TAG, "Sending Data to server...");
			try {
				
				if(reconnect){
					
					if(streamingSocket!=null && !streamingSocket.isClosed())
						streamingSocket.close();
					Log.e(TAG, "Client disconnected, connecting to..." + serverAddress);
					streamingSocket = new Socket(serverAddress, STREAMING_PORT);
					streamingOutput = new DataOutputStream(streamingSocket.getOutputStream());
					streamingInput = new DataInputStream(streamingSocket.getInputStream());
					resolutions=streamingInput.readUTF().split(":");
					//activity.updateResolutionList(resolutions);
					streamingSocket.setKeepAlive(true);
					//activity.getSound().controllerOnline();
					Log.i(TAG, "***Client connected");
					reconnect = false;
				}
				
				while(streamingInput.available()==0){ //maybe this device is going too fast, so wait until there is new data...
                    Thread.sleep(1);
                }
                //Transfering picture
                int arrayLength = streamingInput.readInt();
                picture = new byte[arrayLength];
                streamingInput.readFully(picture);
                //Sending ACK
                streamingOutput.writeBoolean(true);
				
				//Data transfer completed
				requestCompleted = true;				
			} catch (IOException e) {
				Log.e(TAG, "Sudden disconnection from the Server �O� ");
				e.printStackTrace();
				reconnect = true;
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		isTransferingStreaming = false;
		return requestCompleted;
	}
	
	public void updateStreamThread(){
		Thread thread = new Thread() {
            public void run() {
            	boolean update=true;
            	while(update){
            		
            		boolean requestCompleted = false;
            		boolean reconnect = false;
            		isTransferingStreaming = true;
            		int requestNumber = 0;
            		
            		if(streamingSocket==null)
            			reconnect=true;
            		
            		while (!requestCompleted && requestNumber++ < 100){ //100 tries
            			picture=buffPicture;
            			//Log.i(TAG, "Sending Data to server...");
            			try {
            				
            				if(reconnect){
            					
            					if(streamingSocket!=null && !streamingSocket.isClosed())
            						streamingSocket.close();
            					Log.e(TAG, "Client disconnected, connecting to..." + serverAddress);
            					streamingSocket = new Socket(serverAddress, STREAMING_PORT);
            					streamingOutput = new DataOutputStream(streamingSocket.getOutputStream());
            					streamingInput = new DataInputStream(streamingSocket.getInputStream());
            					resolutions=streamingInput.readUTF().split(":");
            					//activity.updateResolutionList(resolutions);
            					streamingSocket.setKeepAlive(true);
            					//activity.getSound().controllerOnline();
            					Log.i(TAG, "***Client connected");
            					reconnect = false;
            				}
            				
            				while(streamingInput.available()==0){ //maybe this device is going too fast, so wait until there is new data...
                                Thread.sleep(1);
                            }
                            //Transfering picture
                            int arrayLength = streamingInput.readInt();
                            buffPicture = new byte[arrayLength];
                            streamingInput.readFully(buffPicture);
                            //Sending ACK
                            streamingOutput.writeBoolean(true);
            				
            				//Data transfer completed
            				requestCompleted = true;				
            			} catch (IOException e) {
            				Log.e(TAG, "Sudden disconnection from the Server �O� ");
            				e.printStackTrace();
            				reconnect = true;
            			} catch (InterruptedException e) {
            				e.printStackTrace();
            			}
            		}
            		isTransferingStreaming = false;
            	}
            }
        };
        thread.start();
	}
	
	public boolean updateController(String msg){
		
		if(isTransferingController)
			return false; //Is still transfering data, skip the next frame.
		
		boolean requestCompleted = false;
		boolean reconnect = false;
		isTransferingController = true;
		int requestNumber = 0;
		
		if(controllerSocket==null)
			reconnect=true;
		
		while (!requestCompleted && requestNumber++ < 100){ //100 tries
			try {
				
				if(reconnect){
					
					if(controllerSocket!=null && !controllerSocket.isClosed())
						controllerSocket.close();
					Log.e(TAG, "Client disconnected, connecting to..." + serverAddress);
					controllerSocket = new Socket(serverAddress, CONTROLLER_PORT);
					controllerOutput = new DataOutputStream(controllerSocket.getOutputStream());
					controllerInput = new DataInputStream(controllerSocket.getInputStream());
					controllerSocket.setKeepAlive(true);
					Log.i(TAG, "Controller connected");
					reconnect = false;
				}
				
				//Sending Controls
                controllerOutput.writeUTF(msg);
                controllerOutput.flush();
				while(controllerInput.available()==0){ //maybe this device is going too fast, so wait until there is new data...
                    Thread.sleep(1);
                }
                //Receiving ACK
                controllerInput.readBoolean();

				//Data transfer completed
				requestCompleted = true;				
			} catch (IOException e) {
				Log.e(TAG, "Sudden disconnection from the controller �O� ");
				e.printStackTrace();
				reconnect = true;
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		isTransferingController = false;
		return requestCompleted;
	}
	
	
	public void shutdown(){
		try {
			if(streamingOutput!=null)
				streamingOutput.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		try {
			if(streamingInput!=null)
				streamingInput.close();
		} catch (IOException e) {
			e.printStackTrace();
		}

		try {
			if(controllerOutput!=null)
				controllerOutput.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		try {
			if(controllerInput!=null)
				controllerInput.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		try {
			if(streamingSocket!=null && !streamingSocket.isClosed())
				streamingSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
			Log.e(TAG, "Client socket not closed properly, check port availability: " + STREAMING_PORT);
		}
		try {
			if(controllerSocket!=null && !controllerSocket.isClosed())
				controllerSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
			Log.e(TAG, "Client socket not closed properly, check port availability: " + CONTROLLER_PORT);
		}
		streamingSocket=null;
		controllerSocket=null;
	}
	
	public String getServerAddress() {
		return serverAddress;
	}

	public void setServerAddress(String serverAddress) {
		this.serverAddress = serverAddress;
	}
	
	public byte[] getPicture() {
		return picture;
	}

	public void setPicture(byte[] picture) {
		this.picture = picture;
	}
	
}