#!/bin/sh

### BEGIN INIT INFO
# Provides:          droneclientd
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: droneclientd service
# Description:       droneclientd manages comms with FC via mavlink
### END INIT INFO

# Change the next 3 lines to suit where you install your script and what you want to call it
DRONEGPRS_PATH=/home/pi/dronegprs-pymavlink
#DIR=$DRONEGPRS_PATH/src
DIR=/usr/bin
DAEMON_NAME=droneclientd
DAEMON=$DIR/python
# Add any command line options for your daemon here
DAEMON_OPTS="$DRONEGPRS_PATH/src/Main.py"

# This next line determines what user the script runs as.
# Root generally not recommended but necessary if you are using the Raspberry Pi GPIO from Python.
DAEMON_USER=pi
HOME_DIR=/home/$DAEMON_USER

#add config option
DAEMON_OPTS="$DAEMON_OPTS --config $HOME_DIR/main.cfg"

# The process ID of the script when it runs is stored here:
PIDFILE=/var/run/$DAEMON_NAME/pid

. /lib/lsb/init-functions

do_start () {
    log_daemon_msg "Starting user $DAEMON_NAME daemon"

    cd $HOME_DIR
    sudo rm -rf /home/pi/modemup
    sudo mkdir -p /var/run/$DAEMON_NAME
    sudo chown $DAEMON_USER:$DAEMON_USER /var/run/$DAEMON_NAME

    set e+
    NOW=$( date '+%F_%H_%M_%S' )
    mv $HOME_DIR/pilot.log $HOME_DIR/pilot.log.$NOW
    set e-
    start-stop-daemon --start --background --pidfile $PIDFILE --make-pidfile --user $DAEMON_USER --chuid $DAEMON_USER:$DAEMON_USER --startas /bin/bash -- -c "exec $DAEMON $DAEMON_OPTS >> $HOME_DIR/pilot.log 2>&1"
    rm -rf /tmp/screenrc.$$
    log_end_msg $?
}
do_stop () {
    log_daemon_msg "Stopping user $DAEMON_NAME daemon"
    start-stop-daemon --stop --remove-pidfile --pidfile $PIDFILE --retry 10
    log_end_msg $?
}

case "$1" in

    start|stop)
        do_${1}
        ;;

    restart|reload|force-reload)
        do_stop
        do_start
        ;;

    status)
        status_of_proc "$DAEMON_NAME" "$DAEMON" && exit 0 || exit $?
        ;;

    *)
        echo "Usage: /etc/init.d/$DAEMON_NAME {start|stop|restart|status}"
        exit 1
        ;;

esac
exit 0

