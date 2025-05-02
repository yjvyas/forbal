## Instructions for running this code

Set BAUD rate as ros params max_dev_from_nominal_ << 0.3, 0.3, 0.1125;etc. in config/params.yaml
Only BAUD rate, Force Divider, Torque Divider, dev name is recognised for now, the code isn't working to set the frequency and filter values!

To run the node with the params in config/params.yaml:
```ros2 launch rft_sensor_serial rft_sensor_launch```
