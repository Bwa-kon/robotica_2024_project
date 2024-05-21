# Get location of this script
SCRIPT="${BASH_SOURCE[0]:-$0}"
REALPATH=`realpath -- "${SCRIPT}"`
SCRIPT_LOC=`dirname "${REALPATH}"`

PS3='Please enter your operating mode: '
options=("With fysical Niryo Ned Robot(Hotspot mode)"
         "With fysical Niryo Ned Robot(Wifi mode)"
         "With fysical Niryo Ned Robot(Ethernet mode)"
         "Without fysical Niryo Ned Robot (simulation Mode)"
         "None")
select opt in "${options[@]}"
do
    case $opt in
        "With fysical Niryo Ned Robot(Hotspot mode)")
            echo "you chose With fysical Niryo Ned Robot(Hotspot mode)"
            source ${SCRIPT_LOC}/hotspot_network.bash
	    break
            ;;
        "With fysical Niryo Ned Robot(Wifi mode)")
            echo "you chose With fysical Niryo Ned Robot(Wifi mode)"
            source ${SCRIPT_LOC}/wifi_network.bash
	    break
            ;;
        "With fysical Niryo Ned Robot(Ethernet mode)")
            echo "you chose With fysical Niryo Ned Robot(Ethernet mode)"
            source ${SCRIPT_LOC}/ethernet_network.bash
	    break
            ;;
        "Without fysical Niryo Ned Robot (simulation Mode)")
            echo "you chose Without fysical Niryo Ned Robot (simulation Mode)"
            source ${SCRIPT_LOC}/simulation.bash
	    break
            ;;
        "None")
            echo "you chose None"
	    break
            ;;
        *) echo "invalid option $REPLY";;
    esac
done
