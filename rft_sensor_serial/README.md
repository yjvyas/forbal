## Instructions for running this code

Set BAUD rate as ros params in config/params.yaml
Only BAUD rate (connection time only), Force Divider, Torque Divider, dev name is recognised for now, the code isn't working to set the frequency and filter values!
To set these and the Baud rate, use the Windows utility.

To run the node with the params in config/params.yaml:
```ros2 launch rft_sensor_serial rft_sensor_launch```
