/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Tabl-O-bot 
****************************************************************************
****************************************************************************
***  Autor: Alejandro Merello
***************************************************************************/
/** \mainpage Tabl-O-Bot documentation
 *  \image html tablobot_logo.png
 *  \section intro_sec Introduction
 *  This is the cell phone's code documentation of project Tabl-O-bot. It is intended for internal use only.
 */
/** \file TablobotClient.java
 *  \brief Bluetooth Client for the Tabl-O-bot.
 *  
 *  Class to handle the Bluetooth connection with the Tabl-O-bot.
 *  \version 1.0
 *  \author Alejandro Merello
 */

import javax.microedition.midlet.*;
import javax.microedition.lcdui.*;

import java.io.IOException;
import java.io.DataInputStream;
import java.io.OutputStream;
import java.util.Timer;
import java.util.TimerTask;

import javax.bluetooth.BluetoothStateException;
import javax.bluetooth.DeviceClass;
import javax.bluetooth.DiscoveryAgent;
import javax.bluetooth.DiscoveryListener;
import javax.bluetooth.LocalDevice;
import javax.bluetooth.RemoteDevice;
import javax.bluetooth.ServiceRecord;
import javax.bluetooth.UUID;
import javax.microedition.io.Connector;
import javax.microedition.io.StreamConnection;

/** This class handles the bluetooth connection and main handling of the application
 */
public class TablobotClient extends MIDlet implements DiscoveryListener,CommandListener{
	static final byte BATT_REQ = 0x20;	///< Battery status request.
	static final byte BT_AUTO = 0x01;	///< Automatic Mode.
	static final byte BT_MANUAL = 0x02;	///< Manual Mode.
	static final byte TB_AUTO = 0x03;	///< Tablobot's change to Automatic Mode.
	static final byte BATT_0 = 0x2B;	///< Battery empty.
	static final byte BATT_25 = 0x2C;	///< Battery 25% load.
	static final byte BATT_50 = 0x2D;	///< Battery 50% load.
	static final byte BATT_75 = 0x2E;	///< Battery 75% load.
	static final byte BATT_100 = 0x2F;	///< Battery full.

	static final String TABLOBOT_address = "00126F01F7C4";                      ///< Tablobot's Bluetooth Address. 00126F01F7DA

	
	protected UUID uuid = new UUID(0x1101);                                     ///< Serial Port Profile
	protected int inquiryMode = DiscoveryAgent.GIAC;                            ///< No limit is set on how long the device remains in the discoverable mode.
	protected int connectionOptions = ServiceRecord.NOAUTHENTICATE_NOENCRYPT;   ///< The Client request no Authentification nor Encryption.
    byte r;                                                                     ///< Stores the bytes received from the Tablobot.
	Timer timer = new Timer();                                                  ///< Timer for battery status request.
	MIDPCanvas buttonCanvas;                                                    ///< Instantiate a Canvas for User Interaction.
    OutputStream out = null;                                                    ///< Handles the output bytes and sends them to the Tabl-O-bot.

	/**
	 * The TablobotClient constructor. Links the buttonCanvas to the current canvas.
	 */
	public TablobotClient() {
		buttonCanvas = new MIDPCanvas(this);
	}

	/**
	 * Performs an action assigned to the Mobile Device - MIDlet Started point.
	 */
	public void startMIDlet() {
	}
	
	/**
	 * Performs an action assigned to the Mobile Device - MIDlet Resumed point.
	 */
	public void resumeMIDlet() {
	}

	/**
	 * Switches a current displayable in a display. The <code>display</code> instance is taken from <code>getDisplay</code> method. This method is used by all actions in the design for switching displayable.
	 * @param alert the Alert which is temporarily set to the display; if <code>null</code>, then <code>nextDisplayable</code> is set immediately.
	 * @param nextDisplayable the Displayable to be set.
	 */
	public void switchDisplayable(Alert alert, Displayable nextDisplayable) {
		Display display = getDisplay();
		if (alert == null) {
			display.setCurrent(nextDisplayable);
		} else {
			display.setCurrent(alert, nextDisplayable);
		}
	}

	/**
	 * Returns a display instance.
	 * @return the display instance.
	 */
	public Display getDisplay () {
		return Display.getDisplay(this);
	}

	/**
	 * Exits MIDlet.
	 */
	public void exitMIDlet() {
		switchDisplayable (null, null);
		try {
			destroyApp(true);
		} catch(Throwable t){log(t);}
		notifyDestroyed();
	}

	/**
	 * Called when MIDlet is started.
	 * Checks whether the MIDlet have been already started and initialize/starts or resumes the MIDlet.
	 */
	protected void startApp() throws MIDletStateChangeException {
		makeGUI();
		startServiceSearch(new RemoteDevice(TABLOBOT_address) {});
	}

	/**
	 * Called when MIDlet is paused.
	 */
	protected void pauseApp() {}

	/**
	 * Called to signal the MIDlet to terminate.
	 * @param arg0 if true, then the MIDlet has to be unconditionally terminated and all resources has to be released.
	 */
	protected void destroyApp(boolean arg0) throws MIDletStateChangeException {}

	/*
	 *   -------  Device inquiry section -------
	 */
	
	/**
	 * Called when a device is found during an inquiry. An inquiry searches for devices that are discoverable. The
	 * same device may be returned multiple times.
	 * @param btDevice The device that was found during the inquiry.
	 * @param cod The service classes, major device class, and minor device class of the remote device.
	 */
	public void deviceDiscovered(RemoteDevice btDevice, DeviceClass cod) {
	}
	
	/**
	 * Called when an inquiry is completed. The discType will be INQUIRY_COMPLETED if the inquiry
	 * ended normally or INQUIRY_TERMINATED if the inquiry was canceled by a call to Discovery-
	 * Agent.cancelInquiry(). The discType will be INQUIRY_ERROR if an error occurred while pro-
	 * cessing the inquiry causing the inquiry to end abnormally.
	 * @param discType The type of request that was completed; either INQUIRY_COMPLETED, INQUIRY_TERMINATED, or INQUIRY_ERROR
	 */
	public void inquiryCompleted(int discType) {}

	/*
	 *   -------  Service search section -------
	 */

	/**
	 * Start search for Serial Port Profile service from device.
	 * @param device The device to search.
	 */
	private void startServiceSearch(RemoteDevice device) {
		try {
			UUID uuids[] = new UUID[] { uuid }; //Start search for Serial Port Profile service from Tablobot
			getAgent().searchServices(null, uuids, device, this);
		} catch (Exception e) {log(e);}
	}

	/**
	 * This method is called when a service(s) are discovered.This method starts
	 * a thread that handles the data exchange with the server.
	 * @param transId The transaction ID of the service search that is posting the result.
	 * @param records A list of services found during the search request.
	 */
	public void servicesDiscovered(int transId, ServiceRecord[] records) {
		for (int i = 0; i < records.length; i++) {
			ServiceRecord rec = records[i];
			String url = rec.getConnectionURL(connectionOptions, false);
			handleConnection(url);
		}
	}

	/**
	 * Called when a service search is completed or was terminated because of an error. Legal status values in the
	 * respCode argument include SERVICE_SEARCH_COMPLETED, SERVICE_SEARCH_TERMINATED, SERVICE_SEARCH_ERROR,
	 * SERVICE_SEARCH_NO_RECORDS, and SERVICE_SEARCH_DEVICE_NOT_REACHABLE.
	 * @param transID The transaction ID identifying the request which initiated the service search.
	 * @param respCode The response code that indicates the status of the transaction.
	 */
	public void serviceSearchCompleted(int transID, int respCode) {
		String msg = null;
		switch (respCode) {
		case SERVICE_SEARCH_COMPLETED: // The service search completed normally
			break;
		case SERVICE_SEARCH_TERMINATED:
			msg = "The service search request was cancelled by a call to DiscoveryAgent.cancelServiceSearch().";
			break;
		case SERVICE_SEARCH_ERROR:
			msg = "An error occurred while processing the request.";
			break;
		case SERVICE_SEARCH_NO_RECORDS:
			msg = "No records were found during the service search.";
			break;
		case SERVICE_SEARCH_DEVICE_NOT_REACHABLE:
			msg = "The Tabl-O-bot could not be reached. Please make sure that the Tabl-O-bot is on and restart the application.";
			break;
		}
        if(msg!=null) log(msg);
	}

	/**
	 * Handles what is done once the connection has been established with the Tablobot
	 * @param url URL Address of the Tablobot
	 */
	private void handleConnection(final String url) {
        Thread echo = new Thread() {
            public void run() {
				StreamConnection stream = null;
				try {
					stream = (StreamConnection) Connector.open(url);
                    stream = new WorkaroundStreamConnection(stream);
					DataInputStream in = stream.openDataInputStream();
					out = stream.openOutputStream();
                    timer.scheduleAtFixedRate(new TimerTask() {public void run() {sendCMD(BATT_REQ);}}, 5000, 20000); //Sends periodically (every 20 [s] a battery update requests to the Tablobot
					while (true) {
						r = in.readByte();
						if((r==TB_AUTO)&&(buttonCanvas.mode!=1)) { buttonCanvas.mode=1; buttonCanvas.repaint(); }
						if((r>=BATT_0)&&(r<=BATT_100)){buttonCanvas.battState=r; buttonCanvas.repaint();}
					}
				} catch (IOException e) {
					log("The Tabl-O-bot was turned off or is out of range.");
				} finally {
					if (stream != null) { // Bluetooth stream closed.
						try {
							stream.close();
						} catch (IOException e) {log(e);}
					}
				}
			}
		};
		echo.start();
	}
	
	/**
	 * Creates an alert dialog box whis is displayed on screen with a certain timeout.
	 * @param msg Message to be displayed.
	 */
	public void log(String msg) {
		Alert alert = new Alert ("Tabl-O-bot");
        alert.setType(AlertType.ERROR);
		alert.setTimeout (10000);
		alert.setString(msg);
		getDisplay().setCurrent(alert);
        alert.setCommandListener(this); //Allows to close the application after an error message
	}

	/**
	 * Creates an alert dialog box of a throwable object.
	 * @param e Object with error message.
	 */
	public void log(Throwable e) {
		log(e.getMessage());
	}
	
	/**
	 * Sets the Canvas
	 */
	private void makeGUI() {
        buttonCanvas.setFullScreenMode(true);
        Display.getDisplay(this).setCurrent(buttonCanvas);
	}

	/**
	 * Returns the discovery agent for this device. Multiple calls to this method will return the same object. This
	 * method will never return <code>null</code>.
	 * @return The discovery agent for the local device.
	 */
	private DiscoveryAgent getAgent() {
		try {
			return LocalDevice.getLocalDevice().getDiscoveryAgent();
		} catch (BluetoothStateException e) {
			throw new Error(e.getMessage());
		}
	}

	/**
	 * Sends a byte through the Bluetooth connection.
	 * @param keyCode Byte to be sent.
	 */
    public void sendCMD(int keyCode){
		try {
			out.write(keyCode);
			out.flush();
			} catch (Exception e) {log(e);}
	}

	/**
	 * Provides the last byte received through Bluetooth connection.
	 * @return The last received byte through the Bluetooth connection.
	 */
	public int getCMD(){
		return r;
	}

	/** Method to retreive the application canvas.
	 * @return Application Canvas.
	 */
	public MIDPCanvas getCanvas(){
		return buttonCanvas;
	}

    /**
     * Indicates that a command event has occurred on Displayable d.
     * @param c A Command object identifying the command.
     * @param d The Displayable on which this event has occurred.
     */
    public void commandAction(Command c, Displayable d){
       if(c==Alert.DISMISS_COMMAND) exitMIDlet();}
}

