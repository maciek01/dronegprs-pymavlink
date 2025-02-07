#!/bin/bash

#exec 1> >(logger -s -t $(basename $0)) 2>&1

#cleanup wifi
sudo rm -rf $HOME/modemup
#sudo ifconfig wlan0 up

#wait for USB
while [ `lsusb |grep Qualcomm|wc -l ` -eq '0' ]
do
    sleep 1
done
echo got modem on a USB port

#modem is up
sleep 1

#start gprs
#MODEM="$($HOME/dronegprs-mavsdk/src/getModem.py)"
MODEM="/dev/ttyUSB2"

echo "Modem id "$MODEM - ${MODEM: -4}
echo $MODEM > $HOME/modemline
if [ "$MODEM" == "" ]; then
	echo "no modem line"
	exit
else
      echo "modem found"
fi

#reset and ping the modem
sudo echo "ATZ" >>$MODEM
sleep 2
sudo echo "AT+CFUN=1,1" >>$MODEM
sleep 2

#dial up
PROVIDER=`grep modemProvider /home/pi/main.cfg`
PROVIDER="$(cut -d'=' -f2 <<<"$PROVIDER")"
echo starting ppp0
pon $PROVIDER-${MODEM: -4}
sleep 1
echo start requested for ppp0

#wait for ppp0
while [ `ifconfig |grep ppp0|wc -l ` -eq '0' ]
do
    sleep 1
done
echo ppp0 available
sleep 5

#capture ppp0 interface ip
IP=`/sbin/ip addr show ppp0 | grep peer | awk ' { print $4 } ' | sed 's/\/32//'`
echo ppp0 ip $IP
echo $IP >$HOME/ppp0-ip

DEFIP=$(/sbin/ip route | awk '/default/ { print $3 }')
echo default route $DEFIP
echo $DEFIP >$HOME/defaultroute

DEFIF=$(/sbin/ip route | awk '/default/ { print $5 }')
echo default if $DEFIF
echo $DEFIF >$HOME/defaultif

#capture wlan0 interface ip and gateway
WLAN_IP=`/sbin/ip addr show wlan0 | grep peer | awk ' { print $4 } ' | sed 's/\/32//'`
echo wlan0 ip $WLAN_IP
echo $WLAN_IP >$HOME/wlan0-ip

WLAN_GW=`/sbin/ip addr show wlan0 | grep peer | awk ' { print $4 } ' | sed 's/\/32//'`
echo wlan0 gw $WLAN_GW
echo $WLAN_GW >$HOME/wlan0-gw

#dont touch dns and routing if wifi is on
if [ -f $HOME/wifi ]; then
    echo "wifi is on - exit and leave default dns and routing in place"
    >$HOME/modemup
    exit
fi

#fixup dns
echo set default DNS via google
sudo cp $HOME/dronegprs-mavsdk/resolv.conf.8.8.8.8 /etc/resolv.conf
sudo cp $HOME/dronegprs-mavsdk/resolv.conf.8.8.8.8 /etc/ppp/resolv.conf

#fixup routing

#default state
#Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
#default         192.168.2.1     0.0.0.0         UG    304    0        0 wlan0
#10.64.64.64     *               255.255.255.255 UH    0      0        0 ppp0
#192.168.2.0     *               255.255.255.0   U     304    0        0 wlan0

set +e
echo set default route via ppp0
#sudo route delete default gw 192.168.2.1 wlan0
sudo route delete default gw $DEFIP $DEFIF
sudo route add default gw $IP ppp0
set -e

#autonomous state
#Destination     Gateway         Genmask         Flags Metric Ref    Use Iface
#default         10.64.64.64     0.0.0.0         UG    0      0        0 ppp0
#10.64.64.64     *               255.255.255.255 UH    0      0        0 ppp0
#192.168.2.0     *               255.255.255.0   U     304    0        0 wlan0

#stop wifi
if [ ! -f $HOME/wifi ]; then
    echo "turn wifi off"
    sudo ifconfig wlan0 down
fi

>$HOME/modemup

