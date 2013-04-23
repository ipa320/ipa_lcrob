package de.fraunhofer.ipa;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.CharBuffer;
import java.nio.channels.ServerSocketChannel;
import java.nio.channels.SocketChannel;
import java.nio.charset.Charset;
import java.nio.charset.CharsetEncoder;

import android.app.Activity;
import android.util.Log;


public class USB_Interface {

	public static class Globals {
		public static boolean connected;
		public static SocketChannel client = null;
	}

	private static final String TAG = "ZeusSocket";
	private ServerSocketChannel server;

	protected USB_Interface() {
		Log.i(TAG, "starting USB interface");
		createSocketServer();
	}

	// socket server needs to do the following things.
	// accept connections until one is established.
	// restart if a connection is lost.
	private void createSocketServer() {
		if (!Globals.connected) {
			Thread t = new Thread(new ServerThread());
			t.start();

			Log.i(TAG, "INITIALIZING SOCKET");
		} else {
			Log.i(TAG, "ALREADY CONNECTED");
		}
	}
	
	public void close() {
		Globals.connected=false;
	}

	public class ServerThread implements Runnable {

		@Override
		public void run() {
			Globals.connected = true;

			try {
				server = ServerSocketChannel.open();
				server.socket().bind(new InetSocketAddress(38300));
				Log.i(TAG, "waiting for connection");
				while (Globals.connected) {
					// listen for incoming clients
					Globals.client = server.accept();
					Globals.client.configureBlocking(false);
					try {
						synchronized(this) {
							mBufferOut="";
							mBufferIn="";
						}
						
						CharsetEncoder enc = Charset.forName("US-ASCII").newEncoder();  

						while(Globals.connected && Globals.client.isConnected()) {

							synchronized(this) {
								if(mBufferOut.length()>0) {
									int written = Globals.client.write(enc.encode(CharBuffer.wrap(mBufferOut)));
									written = written+1;
									mBufferOut = "";
								}
							}

							synchronized(this) {
								ByteBuffer buf = ByteBuffer.allocate(48);
								int bytesRead = Globals.client.read(buf);
								if(bytesRead>0)
									mBufferIn += new String(buf.array());
							}

							try {
								Thread.sleep(20);
							} catch(Exception e) {}

						}

						Log.i(TAG, "closing client");
						Globals.client.close();

						// break;
					} catch (Exception e) {
						Log.i(TAG, "lost client");
						e.printStackTrace();

						Globals.client.close();
					}
				}

			}
			// }
			catch (Exception e) {
				Log.d(TAG, "error, disconnected");
				e.printStackTrace();
			}

			Globals.connected = false;
		}
	};
	private String mBufferOut="";
	private String mBufferIn="";

	public void write(String str) {
		if(Globals.client==null || !Globals.client.isConnected())
			return;

		synchronized(this) {
			mBufferOut += str;
		}
	}

	public String read() {
		String s;
		synchronized(this) {
			s = new String(mBufferIn);
			mBufferIn="";
		}
		return s;
	}

}
