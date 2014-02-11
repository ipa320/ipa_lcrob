/***************************************************************************
***  Fraunhofer IPA 
***  Robotersysteme 
***  Projekt: Tabl-O-bot 
****************************************************************************
****************************************************************************
***  Autor: Alejandro Merello
***************************************************************************/
/** \file MIDPCanvas.java
 *  \brief Canvas for the Cell phone.
 *  
 *  Class to handle the User Interface with the bluetooth program.
 *  \version 1.0
 *  \author Alejandro Merello
 */

import javax.microedition.lcdui.*;

/** This class handles the user interface (Screen and buttons) of the
 * Bluetooth program.
 */
public class MIDPCanvas extends Canvas {
	static final byte BT_AUTO = 0x01;	///< Automatic Mode.
	static final byte BT_MANUAL = 0x02;	///< Manual Mode.
	static final byte TB_AUTO = 0x03;	///< Tablobot's change to Automatic Mode.
	static final byte BATT_0 = 0x2B;	///< Battery empty.
	static final byte BATT_25 = 0x2C;	///< Battery 25% load.
	static final byte BATT_50 = 0x2D;	///< Battery 50% load.
	static final byte BATT_75 = 0x2E;	///< Battery 75% load.
	static final byte BATT_100 = 0x2F;	///< Battery full.

	TablobotClient clientCOMM;	///< Bluetooth connection.

	byte mode=BT_AUTO;			///< Operating mode of Tablobot.
	byte battState=0;			///< Battery state.

	Image ipaLogo = null;			///< Fraunhofer IPA Logo
	Image tablobotLogo = null;		///< Tablobot Logo
	Image tablobotBild =   null;	///< Tablobot Background
	Image batt100 = null;			///< Battery full image.
	Image batt75 = null;			///< Battery 75% image.
	Image batt50 = null;			///< Battery 50% image.
	Image batt25 = null;			///< Battery 25% image.
	Image batt0 = null;				///< Battery empty image.

	/** Constructor sets a link with Bluetooth handling
	 * @param t Tablobot Client with the Bluetooth connection.
	 */
	public MIDPCanvas(TablobotClient t) {
		try {
			clientCOMM = t;
		} catch(Exception e) {clientCOMM.log(e.toString());}
	} 
	
	/** Renders the Canvas.
	 * @param g the Graphics object to be used for rendering the Canvas.
	 */
	public void paint(Graphics g) {
		g.setColor(0,0,0);
		if(tablobotBild==null){
			try {
				tablobotBild = resize(Image.createImage("/tablo.png"),getWidth(),getHeight());
				ipaLogo = Image.createImage("/ipa_logo_90.png");
				tablobotLogo = Image.createImage("/tablobot_100.png");
				batt100 = resize(Image.createImage("/batt_100.png"),getWidth()/4);
				batt75 = resize(Image.createImage("/batt_75.png"),getWidth()/4);
				batt50 = resize(Image.createImage("/batt_50.png"),getWidth()/4);
				batt25 = resize(Image.createImage("/batt_25.png"),getWidth()/4);
				batt0 = resize(Image.createImage("/batt_0.png"),getWidth()/4);
			} catch (java.io.IOException e) { clientCOMM.log("Can't load the image : " + e.toString()); }
        }

		g.drawImage(tablobotBild, 0,0,Graphics.LEFT|Graphics.TOP);
		g.drawImage(ipaLogo, getWidth()-10,10,Graphics.RIGHT|Graphics.TOP);
		g.drawImage(tablobotLogo, getWidth()/2,20+ipaLogo.getHeight(),Graphics.HCENTER|Graphics.TOP);
		if(battState==BATT_100) g.drawImage(batt100, 1,1,Graphics.LEFT|Graphics.TOP);
		else if(battState==BATT_75) g.drawImage(batt75, 1,1,Graphics.LEFT|Graphics.TOP);
			else if(battState==BATT_50) g.drawImage(batt50, 1,1,Graphics.LEFT|Graphics.TOP);
				else if(battState==BATT_25) g.drawImage(batt25, 1,1,Graphics.LEFT|Graphics.TOP);
					else if(battState==BATT_0) g.drawImage(batt0, 1,1,Graphics.LEFT|Graphics.TOP);
        //g.drawString("r " + clientCOMM.getCMD(),0,0,Graphics.LEFT|Graphics.TOP);
		switch(mode){
			case BT_AUTO:		g.drawString("AUTO MODE",getWidth()/2,getHeight()/2,Graphics.HCENTER|Graphics.BASELINE);
							g.setFont(Font.getFont(Font.FACE_PROPORTIONAL, Font.STYLE_BOLD, Font.SIZE_MEDIUM));
							g.setColor(0,255,0);g.drawString("START",6,getHeight()-4,Graphics.LEFT|Graphics.BOTTOM);
							g.setColor(255,0,0);g.drawString("STOP",getWidth()-8,getHeight()-4,Graphics.RIGHT|Graphics.BOTTOM);
							g.setFont(Font.getFont(Font.FACE_PROPORTIONAL, Font.STYLE_PLAIN, Font.SIZE_MEDIUM));
							break;
			case BT_MANUAL:	g.drawString("MANUAL MODE",getWidth()/2,getHeight()/2,Graphics.HCENTER|Graphics.BASELINE); 
							break;
		}
	}
    
	/** Resizes an image to a given width and height.
	 * @param source Source image.
	 * @param width Width of the resized image.
	 * @param height Height of the resized image.
	 * @return A resized image of source.
	 */
	public Image resize(Image source,int width,int height) {
		Image resized = null;
		int rgb[] = new int[source.getWidth()*source.getHeight()]; //Array with the size of the original image
		source.getRGB(rgb,0,source.getWidth(),0,0,source.getWidth(),source.getHeight()); //RGB array of image into "rgb"
		int rgb2[] = reescalaArray(rgb,source.getWidth(),source.getHeight(),width,height); //Call the array resize function
		resized = Image.createRGBImage(rgb2,width,height,true); //Creates image with the RGB array
		return resized;
	}

	/** Resizes in proportion to a given width.
	 * @param source Source image.
	 * @param width Width of the resized image.
	 * @return A proportionally resized image of source.
	 */
	public Image resize(Image source,int width){
		Image resized = null;
		int rgb[] = new int[source.getWidth()*source.getHeight()];
		source.getRGB(rgb,0,source.getWidth(),0,0,source.getWidth(),source.getHeight());
		int rgb2[] = reescalaArray(rgb,source.getWidth(),source.getHeight(),width,source.getHeight()*width/source.getWidth());
		resized = Image.createRGBImage(rgb2,width,source.getHeight()*width/source.getWidth(),true);
		return resized;
	}

	/**
	 * Called when a key is pressed.
	 * @param keyCode The key code of the key that was pressed.
	 */
	protected  void keyPressed(int keyCode) {
		if(clientCOMM.out!=null){
            switch(keyCode){
                case -5:	mode ^= 0x03; clientCOMM.sendCMD(mode); clientCOMM.getCanvas().repaint(); break;
                default:	clientCOMM.sendCMD(keyCode); break;
            }
        }
	}

	/**
	 * Called when a key is released.
	 * @param keyCode The key code of the key that was released.
	 */
	protected  void keyReleased(int keyCode) {
        if(clientCOMM.out!=null){
            /*if(mode==BT_MANUAL)*/ clientCOMM.sendCMD('5');
            clientCOMM.getCanvas().repaint();
        }
	}

	/**
	 * Called when a key is repeated (held down).
	 * @param keyCode The key code of the key that was repeated.
	 */
	protected  void keyRepeated(int keyCode) {
	}

	/**
	 * Called when the pointer is dragged. Not Used.
	 */
	protected  void pointerDragged(int x, int y) {
	}

	/**
	 * Called when the pointer is pressed. Not Used.
	 */
	protected  void pointerPressed(int x, int y) {
	}

	/**
	 * Called when the pointer is released. Not Used.
	 */
	protected  void pointerReleased(int x, int y) {
	}

	/**
	 * Rescales an array.
	 * @param ini Original array.
	 * @param x Width of the source array.
	 * @param y Height of the source array.
	 * @param x2 Width of the resized array.
	 * @param y2 Height of the resized array.
     * @return resized array
	 */
	private int[] reescalaArray(int[] ini, int x, int y, int x2, int y2) {
		int out[] = new int[x2*y2];
		for (int yy = 0; yy < y2; yy++) {
			int dy = yy * y / y2;
			for (int xx = 0; xx < x2; xx++) {
				int dx = xx * x / x2;
				out[(x2*yy)+xx]=ini[(x*dy)+dx];
			}
		}
		return out;
	}
}
