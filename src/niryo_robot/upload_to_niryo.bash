#!/bin/bash

REALPATH=`realpath -- "${0}"`
SCRIPT_LOC=`dirname "${REALPATH}"`

cd "${SCRIPT_LOC}"

./support/upload_robot.bash
