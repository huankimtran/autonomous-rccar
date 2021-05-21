package com.example.uartcontrollertesting.Activity.ThrottleSteerControlPanel;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;

import com.example.uartcontrollertesting.Activity.ThrottleSteerControlPanel.WidgetEventListener.ThrottleSteerSeekbarEventListener;
import com.example.uartcontrollertesting.R;
import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import java.util.HashMap;
import java.util.Iterator;

public class ActivityMain extends AppCompatActivity {
    // UI component references
    TextView tvThrottleValue, tvSteerValue;
    SeekBar sbThrottle, sbSteer;
    ThrottleSteerSeekbarEventListener evtThrottle, evtSteer;
    Button btArdnConnect;

    // USBSerial
    UsbDevice usbDev;
    UsbDeviceConnection usbCon;
    UsbSerialDevice usbSerial;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.throttle_steer_control_panel_layout);
        // Setup the UI
        setupUserInterface();
        // Setup USBSerial
        setupUsbSerialDevice();
    }

    private void setupUserInterface()
    {
        // Components reference
        // Arduino connect button
        btArdnConnect = (Button) findViewById(R.id.hrottleSteerControlPanel_arduino_connect_bt);
        btArdnConnect.setOnClickListener(new View.OnClickListener() {

            @Override
            public void onClick(View v)
            {
                if(usbSerial != null)
                {
                    usbSerial.close();
                }
                setupUsbSerialDevice();
            }
        });
        // Two text view showing value of the seekbars
        tvThrottleValue = (TextView) findViewById(R.id.ThrottleSteerControlPanel_throttle_seekbar_value);
        tvSteerValue = (TextView) findViewById(R.id.ThrottleSteerControlPanel_steer_seekbar_value);
        // The steer and throttle seekbar
        sbThrottle = (SeekBar) findViewById(R.id.ThrottleSteerControlPanel_throttle_seekbar);
        sbSteer = (SeekBar) findViewById(R.id.ThrottleSteerControlPanel_steer_seekbar);
        // Add event listener
        evtThrottle = new ThrottleSteerSeekbarEventListener(tvThrottleValue);
        evtSteer = new ThrottleSteerSeekbarEventListener(tvSteerValue);
        sbThrottle.setOnSeekBarChangeListener(evtThrottle);
        sbSteer.setOnSeekBarChangeListener(evtSteer);
    }

    private void setupUsbSerialDevice()
    {
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        Intent intent = getIntent();
        if(intent.getAction() == Intent.ACTION_MAIN)
        {
            // The user open this app from the launcher so let's perform usb scan to find the arduino nano
            usbDev = scanForArduinoNano();
        }
        else
        {
            usbDev = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        }
        if(usbDev != null)
        {
            // Announce that Arduino has been detected
            popToast("Arduino device found");
            // Get connection to the device
            usbCon = manager.openDevice(usbDev);
            if(usbCon != null)
            {
                popToast("Connection to arduino established");
                usbSerial = UsbSerialDevice.createUsbSerialDevice(usbDev, usbCon);
                if(usbSerial != null)
                {
                    usbSerial.open();
                    usbSerial.setBaudRate(9600);
                    usbSerial.setDataBits(UsbSerialInterface.DATA_BITS_8);
                    usbSerial.setParity(UsbSerialInterface.PARITY_ODD);
                    usbSerial.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
                    evtSteer.setSerialDevice(usbSerial);
                    evtThrottle.setSerialDevice(usbSerial);
                    popToast("Arduino serial interface established, ready!");
                }
            }
            else
            {
                popToast("Error establishing connection");
            }
        }
        else
        {
           popToast("Arduino device NOT found");
        }
    }

    private UsbDevice scanForArduinoNano()
    {
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        HashMap<String, UsbDevice> deviceList = manager.getDeviceList();
        Iterator<UsbDevice> devIter = deviceList.values().iterator();
        while(devIter.hasNext())
        {
            UsbDevice dev = devIter.next();
            if(dev.getVendorId() == 6790 && dev.getProductId() == 29987)
            {
                // Arduino device found
                return dev;
            }
        }
        popToast("Arduino device NOT found");
        return null;
    }

    private void popToast(String message)
    {
        int duration = Toast.LENGTH_LONG;
        Toast.makeText(getBaseContext(), message, duration).show();
    }
}