#!/bin/bash

set -e # exit on error

# PATHS
ROS2_INSTALL_FOLDER_PATH="./ros2_setup_scripts_ubuntu"
ROS2_HUMBLE_PATH="/opt/ros/humble"
WS_DIR=$(pwd)
LOG_PATH="./log"
USER=$(whoami)
ROS_DISTRO="humble"

# CREATE LOG FILE TO SAVE OUTPUT DATA
echo "Creating Log File"
if [ -d "$LOG_PATH" ]; then
    exec > >(tee -a ./log/setup_script_output_$(date +%m%d%Y_%H%M%S).log) 2>&1
    sleep .1 # Sleep command for some reason stops the hanging caused by redirecting output
else
    mkdir -p ./log/
    exec > >(tee -a ./log/setup_script_output_$(date +%m%d%Y_%H%M%S).log) 2>&1
    sleep .1 # Sleep command for some reason stops the hanging caused by redirecting output
fi

# UPDATE AND UPGRADE SYSTEM
sudo apt update -y
sudo apt upgrade -y

# INSTALL ROS2 JAZZY FOR 24.04
echo -e "Install ROS2 Jazzy"

# Check if ROS2 Humble is installed
if [ -d "$ROS2_HUMBLE_PATH" ]; then
    echo -e "ROS2 Humble already installed. Skipping ROS2 installation"
else
    cd ~/

    #Check if ROS2 Install script folder exists and clone if it does not
    if [ -d "$ROS2_INSTALL_FOLDER_PATH" ]; then
        echo "The folder exists. Skipping git clone"
    else
        git clone https://github.com/Tiryoh/ros2_setup_scripts_ubuntu.git
    fi
 
    cd ros2_setup_scripts_ubuntu 
    ./ros2-humble-desktop-main.sh # Install Humble

    cd ../ 
    chmod -R u+w "$ROS2_INSTALL_FOLDER_PATH"
    rm -rv "$ROS2_INSTALL_FOLDER_PATH" #remove install scripts to save space
fi

# INSTALL ROS PACKAGES
echo -e "Installing ROS2 Packages"
sudo apt install ros-$ROS_DISTRO-vision-msgs -y
sudo apt install git -y
sudo apt-get install python3-pip -y
sudo pip3 install --upgrade pip
sudo -H apt-get install python3-pydantic -y
sudo apt-get install ros-humble-usb-cam -y
sudo apt install python3-vcstool -y
sudo apt-get install python3-virtualenv -y
sudo apt-get install v4l-utils -y


# TEXT EDITOR / TOOLS
echo -e "Installing Text Editors and Tools"
sudo apt-get install vim -y
sudo apt-get install gedit -y
sudo apt-get install nano -y
sudo apt-get install emacs -y
sudo apt install terminator -y


# INSTALL FIREWALL
echo -e "Installing and Enabling Firewall"
sudo apt-get install ufw -y
sudo ufw enable
sudo ufw allow from any to any port 22 proto tcp

sudo apt-get install openssh-server
sudo apt-get update
sudo apt-get upgrade

# INSTALL RDP SERVER (Check this)
#https://www.vps-mart.com/blog/install-xfce-and-xrdp-service-on-remote-ubuntu
echo -e "Installing XRDP Server and XFCE4 Desktop Environment"
sudo apt install xfce4 xfce4-goodies -y
sudo apt install xrdp -y
echo "xfce4-session" > ~/.xsession

sudo chmod +x /etc/xrdp/startwm.sh # make startwm.sh executable
sudo cp /etc/xrdp/startwm.sh /etc/xrdp/startwm.sh.orig # backup startwm.sh just in case
sudo cp startwm.sh /etc/xrdp/startwm.sh # copy modified startwm.sh to correct location

sudo systemctl restart xrdp

# check to make sure you are using Xorg not Wayland by running the command below and looking for "x11" in the output
echo -e $XDG_SESSION_TYPE
if [ "$XDG_SESSION_TYPE" = "x11" ]; then
    echo -e "Using Xorg, RDP should work"
else
    echo -e "Not using Xorg, RDP may not work. Run sudo vim /etc/gdm3/custom.conf and make sure WaylandEnable=false is not commented out"
fi

# FIX SNAP 
echo -e "Fixing Snap Issues"
sudo apt install snapd
snap download snapd --revision=24724
sudo snap ack snapd_24724.assert
sudo snap install snapd_24724.snap
sudo snap refresh --hold snapd

# INSTALL ARDUINO IDE and VSCODE
echo -e "Installing IDE and VScode"
sudo snap install arduino

sudo chmod 777 code.deb # make code.deb readable and executable by all users
sudo chown root:root code.deb # change ownership of code.deb to root
echo "code code/add-microsoft-repo boolean true" | sudo debconf-set-selections
sudo dpkg -i code.deb
sudo apt-get install -f 

# ADD TO DIALOUT GROUP TO ACCESS SERIAL PORTS FOR ARDUINO IDE
sudo usermod -a -G dialout $USER

#\ INSTALL FIREFOX
echo -e "Installing Additional Applications"
# sudo apt autoremove -y # optional cleanup?
sudo apt install firefox -y # install firefox for browsing and downloading files
sudo apt install guvcview -y # install guvcview for testing and adjusting camera settings

# ADD UDEV RULES FOR HIWONDER ARM
echo -e "Adding Hiwonder Udev Rules"
sudo cp 99-hiwonder-arm.rules /etc/udev/rules.d/99-hiwonder-arm.rules
sudo udevadm control --reload-rules # reload udev rules to apply changes
sudo chmod +x test_xarm_connection.py # make test_xarm_connection.py executable for testing xarm connection

# SETUP DEPLOY KEYS FOR GITHUB
echo -e "Setting up SSH Keys for GitHub"
ssh-keygen -t ed25519 -C "ME Hand Built Robot Deploy Key for Git Control" -f ~/.ssh/id_ed25519 -N "" # generate new SSH key pair with empty passphrase
eval "$(ssh-agent -s)" # start the ssh-agent in the background
ssh-add ~/.ssh/id_ed25519 # add the private key to the ssh-agent
git config --global user.name "$USER" # set global git username
git config --global user.email "$USER@example.com" # set global git email
git config --global init.defaultBranch main # set default git branch name to main