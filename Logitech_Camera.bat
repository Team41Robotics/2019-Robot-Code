@echo off

echo Logitech Camera
echo.

gst-launch-1.0 -v udpsrc port=5801 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! queue ! decodebin ! videoconvert ! autovideosink 