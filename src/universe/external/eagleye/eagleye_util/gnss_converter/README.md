gnss_converter
==========

A ros package that converts gnss nmea messages to navsatfix messages


# Launch

~~~
source $HOME/ros2_ws/devel/setup.bash
ros2 launch eagleye_gnss_converter gnss_converter.xml
~~~

# Node

## Subscribed Topics
 - /nmea_sentence (nmea_msgs/Sentence)

## Published Topics

 - /fix (sensor_msgs/NavSatFix)

 - /gga (nmea_msgs/Gpgga)

 - /rmc (nmea_msgs/Grmc)


# Parameter description

The parameters are set in `launch/gnss_converter.xml` .

|Name|Type|Description|
|:---|:---|:---|
|gnss.velocity_source_type|int|Selects the velocity input message type.|
|gnss.velocity_source_topic|string|Velocity input topic name.|
|gnss.llh_source_type|int|Selects the latitude/longitude/height input message type.|
|gnss.llh_source_topic|string|Latitude/longitude/height input topic name.|
