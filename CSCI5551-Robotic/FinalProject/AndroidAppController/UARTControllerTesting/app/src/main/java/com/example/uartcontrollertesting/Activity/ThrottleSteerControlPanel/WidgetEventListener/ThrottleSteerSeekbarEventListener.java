package com.example.uartcontrollertesting.Activity.ThrottleSteerControlPanel.WidgetEventListener;

import android.widget.SeekBar;
import android.widget.TextView;

import com.felhr.usbserial.UsbSerialDevice;

public class ThrottleSteerSeekbarEventListener implements SeekBar.OnSeekBarChangeListener
{
    public TextView tvSeekbarValue;
    public UsbSerialDevice serialDev;

    static final String STEER_COMMAND="s %d;";
    static final String THROTTLE_COMMAND="t %d;";

    public ThrottleSteerSeekbarEventListener(TextView txView)
    {
        this.tvSeekbarValue = txView;
    }

    public void setSerialDevice(UsbSerialDevice dev)
    {
        this.serialDev = dev;
    }

    @Override
    public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
    {
        // Adjust the progress to accommodate negative value, the value will be between -100 and 100
        progress -= 100;
        tvSeekbarValue.setText(Integer.toString(progress) + "%");
        // Send the command to the Arduino if avaialbe
        if(serialDev != null)
        {
            if(seekBar.getRotation() > 0.0f)
            {
                // This is the throttle seekbar
                serialDev.write(String.format(THROTTLE_COMMAND, progress).getBytes());
            }
            else
            {
                // This is the steer seekbar
                serialDev.write(String.format(STEER_COMMAND, progress).getBytes());
            }
        }
    }

    @Override
    public void onStartTrackingTouch(SeekBar seekBar)
    {

    }

    @Override
    public void onStopTrackingTouch(SeekBar seekBar)
    {

    }
}
