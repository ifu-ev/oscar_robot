#!/bin/bash
source ~/.bashrc
if ! command -v google-chrome > /dev/null
then
	if ! command -v firefox > /dev/null
	then
		echo "No Web browser can be found."
	else
		firefox $(rospack find oscar_goggles)/web/index.html
	fi
else
	google-chrome --start-fullscreen --kiosk --disable-infobars $(rospack find oscar_goggles)/web/index.html 
fi
