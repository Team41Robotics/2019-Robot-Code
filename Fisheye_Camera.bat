@echo off

echo Fisheye Camera
echo.

gst-launch-1.0 -v udpsrc port=5800 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! queue ! decodebin ! videoconvert ! tee name=t ! queue ! autovideosink t. ! queue ! autovideosink