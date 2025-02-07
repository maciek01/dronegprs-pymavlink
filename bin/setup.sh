
#install pppd for wireless

DRONEGPRS_PATH=/home/pi/dronegprs-mavsdk

sudo rm -rf /etc/ppp/chatscripts
sudo ln -s $DRONEGPRS_PATH/etc/ppp/chatscripts /etc/ppp/chatscripts
sudo ln -s $DRONEGPRS_PATH/etc/ppp/options-mobile /etc/ppp/options-mobile

# LTE-simbase:

sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-simbase-USB2 /etc/ppp/peers/mobile-noauth-simcom7600a-simbase-USB2


# LTE-hologram:

sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-hologram-USB2 /etc/ppp/peers/mobile-noauth-simcom7600a-hologram-USB2

# LTE-att

sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-att-USB0 /etc/ppp/peers/mobile-noauth-simcom7600a-att-USB0
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-att-USB1 /etc/ppp/peers/mobile-noauth-simcom7600a-att-USB1
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-att-USB2 /etc/ppp/peers/mobile-noauth-simcom7600a-att-USB2
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom7600a-att-USB3 /etc/ppp/peers/mobile-noauth-simcom7600a-att-USB3

# 3G-hologram:

sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom5320a-hologram-USB2 /etc/ppp/peers/mobile-noauth-simcom5320a-hologram-USB2

# 3G-att:

sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom5320a-att-USB0 /etc/ppp/peers/mobile-noauth-simcom5320a-att-USB0
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom5320a-att-USB1 /etc/ppp/peers/mobile-noauth-simcom5320a-att-USB1
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom5320a-att-USB2 /etc/ppp/peers/mobile-noauth-simcom5320a-att-USB2
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/mobile-noauth-simcom5320a-att-USB3 /etc/ppp/peers/mobile-noauth-simcom5320a-att-USB3

# 2G:
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/fonaUSB0 /etc/ppp/peers/fonaUSB0
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/fonaUSB1 /etc/ppp/peers/fonaUSB1
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/fonaUSB2 /etc/ppp/peers/fonaUSB2
sudo ln -s $DRONEGPRS_PATH/etc/ppp/peers/fonaUSB3 /etc/ppp/peers/fonaUSB3

#install client daemon

sudo ln -s $DRONEGPRS_PATH/bin/droneclientd.sh /etc/init.d/droneclientd
sudo ln -s $DRONEGPRS_PATH/bin/fonad.sh /etc/init.d/fonad
sudo update-rc.d droneclientd defaults
sudo update-rc.d fonad defaults

sudo usermod -a -G dip pi
#sudo usermod -G pi,adm,dialout,cdrom,sudo,audio,video,plugdev,games,users,input,netdev,gpio,i2c,spi,dip pi

