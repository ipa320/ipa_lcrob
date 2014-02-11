/** \file WorkaroundStreamConnection.java
 *  \brief Reliable stream connection solution for some cell phone models.
 *
 *  \author 2006 Nokia Corporation.
 *  \htmlonly
 *  <A HREF="http://www.forum.nokia.com/info/sw.nokia.com/id/85b9d2ee-83f1-4c7b-8e71-0b1e35d0c1e9/PC_Connectivity_over_BT_in_Java_Apps_with_Example_v1_0_en.zip.html"> PC Connectivity over BT in Java Apps with Example </A>
 *	\endhtmlonly
 * 
 */

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

import javax.microedition.io.StreamConnection;

public class WorkaroundStreamConnection extends Object implements StreamConnection {

	private static final int SLOW_DOWN = 10; // ms

	private StreamConnection conn;

	public WorkaroundStreamConnection(StreamConnection stream) {
		this.conn = stream;
	}

	public InputStream openInputStream() throws IOException {
		return new InputStream() {
			InputStream in = conn.openInputStream();

			public synchronized int read() throws IOException {
				int ret = -1;
				while (true) {
					ret = in.read();
					if (ret < 0) {
						try {
							Thread.sleep(SLOW_DOWN);
						} catch (InterruptedException e) {
						}
						continue;
					}
					break;
				}
				try {
					Thread.sleep(SLOW_DOWN);
				} catch (InterruptedException e) {
				}
				return ret;
			}
		};
	}

	public DataInputStream openDataInputStream() throws IOException {
		return new DataInputStream(openInputStream());
	}

	public void close() throws IOException {
		conn.close();
	}

	public OutputStream openOutputStream() throws IOException {
		return new OutputStream() {
			OutputStream out = conn.openOutputStream();

			public synchronized void write(int value) throws IOException {
				out.write(value);
				out.flush();
				try {
					Thread.sleep(SLOW_DOWN);
				} catch (InterruptedException e) {
				}
			}
			
			public void flush() throws IOException {
			}
		};
	}

	public DataOutputStream openDataOutputStream() throws IOException {
		return new DataOutputStream(openOutputStream());
	}

}
