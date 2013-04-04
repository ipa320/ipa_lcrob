package de.fraunhofer.ipa;

import java.util.Calendar;

import com.example.extendeddevice.R;

import android.app.Activity;
import android.app.AlarmManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.View;
import android.view.View.OnKeyListener;
import android.view.Window;
import android.view.WindowManager;

public class Main extends Activity {
	
	static Activity act_ = null;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, 
                                WindowManager.LayoutParams.FLAG_FULLSCREEN);

		setContentView(R.layout.activity_main);
		
		findViewById(R.id.imageView1).setOnKeyListener(new OnKeyListener() {

			@Override
			public boolean onKey(View v, int keyCode, KeyEvent event) {
			    return true;
			}
			
		});
		
		act_ = this;
		
		Log.i("MAIN", "main window started");
		
		startService(new Intent(this, IMU_Service.class));

		/*Intent intent = new Intent(this, IMU_Service.class);
		PendingIntent pintent = PendingIntent.getService(this, 0, intent, 0);

		AlarmManager alarm = (AlarmManager)getSystemService(Context.ALARM_SERVICE);

		// Start every 30 ms
		alarm.setRepeating(AlarmManager.RTC_WAKEUP, Calendar.getInstance().getTimeInMillis(), 30, pintent);*/
	}
	
	@Override
    public void onBackPressed() { //disable back button in this activity
        return;
    }
	
	@Override
	public void onDestroy() {
		super.onDestroy();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		return true;
	}
} 
