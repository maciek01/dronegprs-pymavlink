

#run this in anothe terminal: cat /dev/ttyUSB0

echo "AT" >>/dev/ttyUSB0
sleep 1

echo "AT" >>/dev/ttyUSB0
sleep 1


echo "ATI" >>/dev/ttyUSB0
sleep 1


echo "AT+CMEE=2" >>/dev/ttyUSB0
sleep 1


echo "AT+CCID" >>/dev/ttyUSB0
sleep 1


echo "AT+COPS?" >>/dev/ttyUSB0
sleep 1


echo "AT+CSQ" >>/dev/ttyUSB0
sleep 1


echo "AT+CBC" >>/dev/ttyUSB0
sleep 1


echo "ATD16172816327;" >>/dev/ttyUSB0
sleep 15


echo "ATH" >>/dev/ttyUSB0

