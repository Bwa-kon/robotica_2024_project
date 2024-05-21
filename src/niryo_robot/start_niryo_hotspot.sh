# Get location of this script
SCRIPT="${BASH_SOURCE[0]:-$0}"
REALPATH=`realpath -- "${SCRIPT}"`
SCRIPT_LOC=`dirname "${REALPATH}"`

source "${SCRIPT_LOC}/support/hotspot_network.bash"
