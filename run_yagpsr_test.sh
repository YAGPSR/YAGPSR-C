#!/bin/sh
./yagpsr -DopplerSwing 5000 -DopplerStep 250 -DetectionThreshold 150 -CorrelationWindow 100 -SleepTime 10000 -fTestGPS -CorrelationMode FFT
mv LatLonGPS_YAGPSR.kml TestGPS_Lat_Lon.kml
echo "\n\nTest results: TestGPS_Lat_Lon.kml\n\n"