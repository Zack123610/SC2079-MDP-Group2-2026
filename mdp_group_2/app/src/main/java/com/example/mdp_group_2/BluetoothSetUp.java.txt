package com.example.mdp_group_14;

import android.Manifest;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Gravity;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.ListView;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;


public class BluetoothSetUp extends Fragment {

    private static final String TAG = "Bluetooth Debug";

    private String connStatus;
    BluetoothAdapter mBluetoothAdapter;
    public ArrayList<BluetoothDevice> mNewBTDevices = new ArrayList<>();
    public ArrayList<BluetoothDevice> mPairedBTDevices = new ArrayList<>();
    public DeviceListAdapter mNewDeviceListAdapter;
    public DeviceListAdapter mPairedDeviceListAdapter;
    public static TextView connStatusTextView;
    ListView lvNewDevices;  // otherDevicesListView
    ListView lvPairedDevices;   // pairedDevicesListView
    Button connectBtn;
    Button btnSearch;
    ProgressDialog myDialog;

    private static String[] PERMISSIONS_STORAGE = {
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.WRITE_EXTERNAL_STORAGE,
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_LOCATION_EXTRA_COMMANDS,
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
    };
    private static String[] PERMISSIONS_LOCATION = {
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION,
            Manifest.permission.ACCESS_LOCATION_EXTRA_COMMANDS,
            Manifest.permission.BLUETOOTH_SCAN,
            Manifest.permission.BLUETOOTH_CONNECT,
    };

    SharedPreferences sharedPreferences;
    SharedPreferences.Editor editor;

    BluetoothConnectionService mBluetoothConnection;
    private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    public static BluetoothDevice mBTDevice;

    boolean retryConnection = false;
    Handler reconnectionHandler = new Handler();
    Context mContext;

    Runnable reconnectionRunnable = new Runnable() {
        @Override
        public void run() {
            try {
                if (mBTDevice != null && BluetoothConnectionService.BluetoothConnectionStatus == false) {
                    showLog("Reconnecting...");
                    startBTConnection(mBTDevice, MY_UUID);
                }
                // Schedule the next check/attempt in 5 seconds
                reconnectionHandler.postDelayed(this, 5000);
            } catch (Exception e) {
                showLog("Reconnection error");
                e.printStackTrace();
                reconnectionHandler.postDelayed(this, 5000);
            }
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
    }


    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState)
    {
        // inflate
        View root = inflater.inflate(R.layout.bluetooth, container, false);
        // get shared preferences
        sharedPreferences = getActivity().getSharedPreferences("Shared Preferences",
                Context.MODE_PRIVATE);


        DisplayMetrics dm = new DisplayMetrics();
        getActivity().getWindowManager().getDefaultDisplay().getMetrics(dm);

        lvNewDevices = root.findViewById(R.id.otherDevicesListView);
        lvPairedDevices = root.findViewById(R.id.pairedDevicesListView);
        mNewBTDevices = new ArrayList<>();
        mPairedBTDevices = new ArrayList<>();

        connectBtn = root.findViewById(R.id.connectBtn);
        btnSearch = root.findViewById(R.id.scanButton);

        int width = dm.widthPixels;
        int height = dm.heightPixels;

        // Get bluetooth adapter
        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        //finds the bluetooth switch
        Switch bluetoothSwitch = root.findViewById(R.id.bluetoothSwitch);
        if(mBluetoothAdapter.isEnabled()){
            bluetoothSwitch.setChecked(true);
            bluetoothSwitch.setText("ON");
        }

        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        getActivity().registerReceiver(mBroadcastReceiver4, filter);

        IntentFilter filter2 = new IntentFilter("ConnectionStatus");
        LocalBroadcastManager.getInstance(getActivity()).registerReceiver(mBroadcastReceiver5, filter2);

        lvNewDevices.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
                mBluetoothAdapter.cancelDiscovery();
                lvPairedDevices.setAdapter(mPairedDeviceListAdapter);

                String deviceName = mNewBTDevices.get(i).getName();
                String deviceAddress = mNewBTDevices.get(i).getAddress();
                Log.d(TAG, "onItemClick: A device is selected.");
                Log.d(TAG, "onItemClick: DEVICE NAME: " + deviceName);
                Log.d(TAG, "onItemClick: DEVICE ADDRESS: " + deviceAddress);

                if (Build.VERSION.SDK_INT > Build.VERSION_CODES.JELLY_BEAN_MR2) {
                    Log.d(TAG, "onItemClick: Initiating pairing with " + deviceName);
                    mNewBTDevices.get(i).createBond();

                    mBluetoothConnection = new BluetoothConnectionService(getContext());
                    mBTDevice = mNewBTDevices.get(i);
                }
            }
        });

        lvPairedDevices.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
                mBluetoothAdapter.cancelDiscovery();
                lvNewDevices.setAdapter(mNewDeviceListAdapter);

                String deviceName = mPairedBTDevices.get(i).getName();
                String deviceAddress = mPairedBTDevices.get(i).getAddress();
                Log.d(TAG, "onItemClick: A device is selected.");
                Log.d(TAG, "onItemClick: DEVICE NAME: " + deviceName);
                Log.d(TAG, "onItemClick: DEVICE ADDRESS: " + deviceAddress);

                mBluetoothConnection = new BluetoothConnectionService(getContext());
                mBTDevice = mPairedBTDevices.get(i);
            }
        });


        bluetoothSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener(){
            // Enabling and Disabling of Bluetooth on The Device
            @Override
            public void onCheckedChanged(CompoundButton compoundButton, boolean isChecked) {
                Log.d(TAG, "onChecked: Enabling/Disabling Bluetooth");
                if(isChecked){
                    compoundButton.setText("ON");
                }else
                {
                    compoundButton.setText("OFF");
                }

                if (mBluetoothAdapter == null) {
                    Log.d(TAG, "enableDisableBT: Does not have Bluetooth capabilities");
                    updateStatus("Error: Bluetooth is not supported on this device");
                    compoundButton.setChecked(false);
                }
                else {
                    if (!mBluetoothAdapter.isEnabled()) {
                        Log.d(TAG, "enableDisableBT: Enabling Bluetooth");

                        Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
                        discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
                        startActivity(discoverableIntent);

                        compoundButton.setChecked(true);

                        //IntentFilter catches the state change
                        IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
                        getActivity().registerReceiver(mBroadcastReceiver1, BTIntent);
                    }
                    if (mBluetoothAdapter.isEnabled()) {
                        Log.d(TAG, "enableDisableBT: Disabling Bluetooth");
                        mBluetoothAdapter.disable();

                        IntentFilter BTIntent = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
                        getActivity().registerReceiver(mBroadcastReceiver1, BTIntent);
                    }
                }
            }
        });

        connectBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(mBTDevice ==null)
                {
                    updateStatus("Please Select a Device before connecting.");
                }
                else {
                    startConnection();
                }
            }
        });
        //ONCLICKLISTENER FOR SEARCH BUTTON
        btnSearch.setOnClickListener(new View.OnClickListener() {
            public void onClick(View view) {
                Log.d(TAG, "onClick: search button");
                toggleButtonScan(view);
            }
        });

        Button backBtn = root.findViewById(R.id.backBtn);

        connStatusTextView = root.findViewById(R.id.connStatusTextView);
        connStatus ="Disconnected";
        sharedPreferences = getActivity().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
        if (sharedPreferences.contains("connStatus"))
            connStatus = sharedPreferences.getString("connStatus", "");

        connStatusTextView.setText(connStatus);

        backBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                editor = sharedPreferences.edit();
                editor.putString("connStatus", connStatusTextView.getText().toString());
                editor.commit();
                getActivity().finish();
            }
        });

        myDialog = new ProgressDialog(getContext());
        myDialog.setMessage("Waiting for other device to reconnect...");
        myDialog.setCancelable(false);
        myDialog.setButton(DialogInterface.BUTTON_NEGATIVE, "Cancel", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
                // Stop retry if user cancels
                reconnectionHandler.removeCallbacks(reconnectionRunnable);
                retryConnection = false;
            }
        });

        return root;
    }

    private void checkBTPermissions(){
        int permission1 = ActivityCompat.checkSelfPermission(getActivity(), Manifest.permission.WRITE_EXTERNAL_STORAGE);
        int permission2 = ActivityCompat.checkSelfPermission(getActivity(), Manifest.permission.BLUETOOTH_SCAN);
        if (permission1 != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(getActivity(), PERMISSIONS_STORAGE, 1);
        } else if (permission2 != PackageManager.PERMISSION_GRANTED){
            ActivityCompat.requestPermissions(getActivity(), PERMISSIONS_LOCATION, 1);
        }
    }

    public void Scanning() {
        Log.d(TAG, "toggleButton: Scanning for unpaired devices.");
        checkBTPermissions();
        mNewBTDevices.clear();
        if (mBluetoothAdapter != null) {
            if (!mBluetoothAdapter.isEnabled()) {
                updateStatus( "Please turn on Bluetooth first!");
            }
            if (mBluetoothAdapter.isDiscovering()) {
                mBluetoothAdapter.cancelDiscovery();
                mBluetoothAdapter.startDiscovery();
                IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND);
                getActivity().registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
            }
            else if (!mBluetoothAdapter.isDiscovering()) {
                mBluetoothAdapter.startDiscovery();
                IntentFilter discoverDevicesIntent = new IntentFilter(BluetoothDevice.ACTION_FOUND);
                getActivity().registerReceiver(mBroadcastReceiver3, discoverDevicesIntent);
            }
            mPairedBTDevices.clear();
            Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
            for(BluetoothDevice d : pairedDevices){
                mPairedBTDevices.add(d);
                mPairedDeviceListAdapter = new DeviceListAdapter(getContext(), R.layout.device_adapter_view, mPairedBTDevices);
                lvPairedDevices.setAdapter(mPairedDeviceListAdapter);
            }
        }
    }

    public void toggleButtonScan(View view){
        Scanning();
    }

    private final BroadcastReceiver mBroadcastReceiver1 = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (action.equals(mBluetoothAdapter.ACTION_STATE_CHANGED)) {
                final int state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE,mBluetoothAdapter.ERROR);
                switch(state){
                    case BluetoothAdapter.STATE_OFF:
                        Log.d(TAG, "onReceive: STATE OFF");
                        break;
                    case BluetoothAdapter.STATE_ON:
                        Log.d(TAG, "onReceive: STATE ON");
                        break;
                }
            }
        }
    };

    private final BroadcastReceiver mBroadcastReceiver2 = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (action.equals(BluetoothAdapter.ACTION_SCAN_MODE_CHANGED)) {
                final int mode = intent.getIntExtra(BluetoothAdapter.EXTRA_SCAN_MODE, BluetoothAdapter.ERROR);
                switch (mode) {
                    case BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE:
                        Log.d(TAG, "mBroadcastReceiver2: Discoverability Enabled.");
                        break;
                }
            }
        }
    };

    private final BroadcastReceiver mBroadcastReceiver3 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if(action.equals(BluetoothDevice.ACTION_FOUND)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if(device.getBondState()!=BluetoothDevice.BOND_BONDED) {
                    mNewBTDevices.add(device);
                    mNewDeviceListAdapter = new DeviceListAdapter(context, R.layout.device_adapter_view, mNewBTDevices);
                    lvNewDevices.setAdapter(mNewDeviceListAdapter);
                }
            }
        }
    };

    private final BroadcastReceiver mBroadcastReceiver4 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if(action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED)){
                BluetoothDevice mDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if(mDevice.getBondState() == BluetoothDevice.BOND_BONDED){
                    updateStatus("Successfully paired with " + mDevice.getName());
                    Scanning();
                }
            }
        }
    };

    private final BroadcastReceiver mBroadcastReceiver5 = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            BluetoothDevice mDevice = intent.getParcelableExtra("Device");
            String status = intent.getStringExtra("Status");
            sharedPreferences = getActivity().getSharedPreferences("Shared Preferences", Context.MODE_PRIVATE);
            editor = sharedPreferences.edit();

            if(status.equals("connected")){
                try {
                    myDialog.dismiss();
                } catch(NullPointerException e){
                    e.printStackTrace();
                }
                // Stop retry loop on success
                reconnectionHandler.removeCallbacks(reconnectionRunnable);
                retryConnection = false;

                Log.d(TAG, "mBroadcastReceiver5: Device now connected to "+mDevice.getName());
                updateStatus( "Device now connected to "+mDevice.getName());
                editor.putString("connStatus", "Connected to " + mDevice.getName());
                connStatusTextView.setText("Connected to " + mDevice.getName());
            }
            else if(status.equals("disconnected") && retryConnection == false){
                Log.d(TAG, "mBroadcastReceiver5: Disconnected from "+mDevice.getName());
                updateStatus("Disconnected from "+mDevice.getName());
                
                editor.putString("connStatus", "Disconnected");
                connStatusTextView.setText("Disconnected");
                editor.commit();

                try {
                    myDialog.show();
                }catch (Exception e){
                    Log.d(TAG, "Dialog show failure");
                }
                
                // Start indefinite retry loop
                retryConnection = true;
                reconnectionHandler.postDelayed(reconnectionRunnable, 2000);
            }
            editor.commit();
        }
    };

    public void startConnection(){
        startBTConnection(mBTDevice,MY_UUID);
    }

    public void startBTConnection(BluetoothDevice device, UUID uuid){
        if (mBluetoothConnection == null) {
            mBluetoothConnection = new BluetoothConnectionService(getContext());
        }
        mBluetoothConnection.startClientThread(device, uuid);
    }


    @Override
    public void onDestroy() {
        super.onDestroy();
        try {
            getActivity().unregisterReceiver(mBroadcastReceiver1);
            getActivity().unregisterReceiver(mBroadcastReceiver2);
            getActivity().unregisterReceiver(mBroadcastReceiver3);
            getActivity().unregisterReceiver(mBroadcastReceiver4);
            LocalBroadcastManager.getInstance(getContext()).unregisterReceiver(mBroadcastReceiver5);
            reconnectionHandler.removeCallbacks(reconnectionRunnable);
        } catch(Exception e){
            e.printStackTrace();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        try {
            getActivity().unregisterReceiver(mBroadcastReceiver1);
            getActivity().unregisterReceiver(mBroadcastReceiver2);
            getActivity().unregisterReceiver(mBroadcastReceiver3);
            getActivity().unregisterReceiver(mBroadcastReceiver4);
            LocalBroadcastManager.getInstance(getContext()).unregisterReceiver(mBroadcastReceiver5);
            reconnectionHandler.removeCallbacks(reconnectionRunnable);
        } catch(Exception e){
            e.printStackTrace();
        }
    }

    private void showLog(String message) { Log.d(TAG, message); }
    private void updateStatus(String message) {
        Toast toast = Toast.makeText(getContext(), message, Toast.LENGTH_SHORT);
        toast.setGravity(Gravity.TOP,0, 0);
        toast.show();
    }
}
