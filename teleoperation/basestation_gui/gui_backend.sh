#!/bin/bash



cd teleoperation/basestation_gui || exit 1
# clears the currentautoncourse table upon restarting the app
python3 manage.py migrate
python3 manage.py runserver