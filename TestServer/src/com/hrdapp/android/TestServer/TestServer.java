package com.hrdapp.android.TestServer;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import android.net.LocalServerSocket ;
import android.net.LocalSocket;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.Toast;


public class TestServer extends Activity {
    private Handler handler;
    private SocketListener thread;

    public void showNotification(String message) {
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show();
    }
    
    class SocketListener extends Thread {
        private Handler handler = null;
        private LocalSocket connectsocket = null;
        
        public SocketListener(Handler handler) {
            this.handler = handler;
        }

        private void receivcMessage(String message) {
            Log.v(getClass().getName(), "receivcMessage");
        	Message msg = new Message();
			msg.obj = message; 
            this.handler.sendMessage(msg);
        }

        private void sendMessage(String message) {
            Log.v(getClass().getName(), "sendMessage");
        	if(connectsocket == null)
        		return;
        	try {
				OutputStream output = connectsocket.getOutputStream();
				output.write(message.getBytes());
			} catch (IOException e) {
                Log.e(getClass().getName(), e.getMessage());
			}
        }

        public void Disconnect() {
            Log.v(getClass().getName(), "Disconnect");
        	if(connectsocket == null)
        		return;
        	
        	try {
				connectsocket.close();
			} catch (IOException e) {
                Log.e(getClass().getName(), e.getMessage());
			}
        	connectsocket = null;
        }

        @Override
        public void run() {
            Log.v(getClass().getName(), "run");
            try {
                LocalServerSocket server = new LocalServerSocket("adbpic24f");
                while (true) {
                    LocalSocket socket = server.accept();
                    if (socket != null) {
                        Log.v(getClass().getName(), "accept");
                    	connectsocket = socket;
                        InputStream input = socket.getInputStream();

                        int readed;
                        byte[] bytes = new byte[32];
                        
                        // reading
                        while (true)
                        {
                            readed = input.read(bytes,0,32);
                            if(readed > 0)
                            {
                            	receivcMessage(new String(bytes, 0, readed));
                            }
                        }
                    }
                }
            } catch (IOException e) {
                Log.e(getClass().getName(), e.getMessage());
            }
            this.stop();
        }
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        
        handler = new Handler() 
        {
			public void handleMessage(android.os.Message msg) {
				EditText ReceiveText = (EditText) findViewById(R.id.editText2);
				ReceiveText.setText((String) msg.obj);

				showNotification((String) msg.obj);
			};      	
        };

        thread = new SocketListener(this.handler);
        thread.start();
    }

    public void onDestroy ()
    {
    	thread.Disconnect();
    	super.onDestroy();
    }
    
    
	public void onClickSend(View view){
    	String text;
		EditText sendText = (EditText) findViewById(R.id.editText1);

    	text = sendText.getText().toString();

    	thread.sendMessage(text);
	}
}