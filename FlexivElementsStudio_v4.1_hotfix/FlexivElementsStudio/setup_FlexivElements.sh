#!/bin/sh
# default, setup as FlexivElementsStudio
IS_STUDIO=true
UI="FlexivElements"
STUDIO="FlexivElementsStudio"
UI_NAME=${STUDIO}
UI_PLATFORM="ubuntu_pc"
TMOUT=30

# ==============================================================================
# Get password and check password
# ==============================================================================
Get_password_and_check() {
# Check if user has NOPASSWD sudo access
NEED_PASSWORD=true
if sudo -n true 2>/dev/null; then
	NEED_PASSWORD=false
	echo "[NOTICE] Detected NOPASSWD sudo configuration... !"
fi

# Get password if need password
if [ "$NEED_PASSWORD" = true ]; then
	# Check whether there is run_FlexivElements.sh
	if [ -e ./run_FlexivElements.sh ]; then
		echo "Exist run_FlexivElements.sh"
		# check whether run_FlexivElements.sh is empty
		if [ ! -s ./run_FlexivElements.sh ]; then
			echo  "[WARNING] run_FlexivElements.sh file is empty. Please remove it and run setup_FlexivElements.sh first."
			exit 1;
		fi
		# Get password from script
		extracted_line=$(grep '^sudo -k && echo' run_FlexivElements.sh)
		txt_remove_suffix=$(echo "$extracted_line" | awk -F'|' '{print $1}')
		txt_remove_prefix=$(echo "$txt_remove_suffix" | awk -F'echo' '{print $2}')
		PASSWORD_WITH_MARK=$(echo "$txt_remove_prefix" | awk '{gsub(/ /, ""); print}')
		PASSWORD=$(echo "$PASSWORD_WITH_MARK" | awk -F"'" '{print $2}')
	else
		# Get password from user input
		echo "Please enter $(whoami) password:"
		if ! read -s PASSWORD; then
			echo "[WARNING] Timeout reached. Exiting script... !"
			exit 1
		fi
	fi

	# Escape single quotes in password for safe passing
	sudo -k && echo ${PASSWORD} | sudo -S su > /dev/null 2>&1
	if [ $? -ne 0 ];then
		echo "Incorrect password. Please follow the suggestions according to the current case."
		echo "  case 1: In first setup process, please try again;"
		echo "  case 2: In other setup process, please remove run_FlexivElements.sh and run setup_FlexivElements.sh again;"
		echo "  case 3: In version update process, please remove run_FlexivElements.sh,run setup_FlexivElements.sh first and update UI again;"
		exit 1;
	fi

	echo "[DONE] Get & Check password... !"
fi
}

# ==============================================================================
# Create startup Flexiv Elements script
# ==============================================================================
Create_run_script() {
# Switch user
sudo chown -R ${USERNAME}:${USERNAME} ./* ./specs_ui* ./user_data_ui*

#!! need check password for sudo
if [ "$NEED_PASSWORD" = true ]; then
cat << EOE > run_FlexivElements.sh
#!/bin/sh
# Kill GUI
pkill -9 FlexivElements

# Setting
CLD_LIBRARY_PATH=\`pwd \`"/lib"
QT_QPA_PLATFORM_PLUGIN_PATH=\`pwd \`"/plugins"

# Add permissions
sudo -k && echo '${PASSWORD}' | sudo -S su > /dev/null 2>&1
if [ \$? -ne 0 ];then
	echo "Failed to obtain sudo permissions"
	exit 1;
fi

# General core dump file config
sudo bash -c "echo FlexivElements.core > /proc/sys/kernel/core_pattern"
ulimit -c unlimited

# Remove permissions
sudo -k

# we need unset network http proxy
unset http_proxy
unset https_proxy

# Startup GUI
LD_LIBRARY_PATH="\${CLD_LIBRARY_PATH}":\${LD_LIBRARY_PATH} QT_QPA_PLATFORM_PLUGIN_PATH="\${QT_QPA_PLATFORM_PLUGIN_PATH}" QTWEBENGINE_DISABLE_SANDBOX=1 \\
./${UI_NAME} -p ${UI_PLATFORM}
EOE

else

#!! no need check password for sudo
cat << EOE > run_FlexivElements.sh
#!/bin/sh
# Kill GUI
pkill -9 FlexivElements

# Setting
CLD_LIBRARY_PATH=\`pwd \`"/lib"
QT_QPA_PLATFORM_PLUGIN_PATH=\`pwd \`"/plugins"

# Add permissions
if ! sudo true 2>/dev/null; then
	echo "Failed to obtain sudo permissions"
	exit 1;
fi

# General core dump file config
sudo bash -c "echo FlexivElements.core > /proc/sys/kernel/core_pattern"
ulimit -c unlimited

# we need unset network http proxy
unset http_proxy
unset https_proxy

# Startup GUI
LD_LIBRARY_PATH="\${CLD_LIBRARY_PATH}":\${LD_LIBRARY_PATH} QT_QPA_PLATFORM_PLUGIN_PATH="\${QT_QPA_PLATFORM_PLUGIN_PATH}" QTWEBENGINE_DISABLE_SANDBOX=1 \\
./${UI_NAME} -p ${UI_PLATFORM}
EOE
fi

echo "[DONE] Create run_FlexivElements.sh... !"
}

# ==============================================================================
# Create Flexiv Elements shortcuts, Flexiv Elements Automatic login shortcuts
# ==============================================================================
Create_Flexiv_Elements_shortcuts() {
# Create Flexiv Elements shortcuts
FLEXIV_ELEMENTS_SC=$HOME"/Desktop/${UI_NAME}.desktop"     # Flexiv Elements desktop shortcuts file
FLEXIV_ELEMENTS_PATH=$(pwd)                                   # Flexiv Elements run path
FLEXIV_ELEMENTS_EXEC=$(pwd)"/run_FlexivElements.sh"           # Flexiv Elements run Script

## Create Flexiv Elements desktop shortcuts file
rm -rf "${FLEXIV_ELEMENTS_SC}"
touch "${FLEXIV_ELEMENTS_SC}"
echo "[Desktop Entry]" >> "${FLEXIV_ELEMENTS_SC}"
echo "Encoding=UTF-8" >> "${FLEXIV_ELEMENTS_SC}"
echo "Type=Application" >> "${FLEXIV_ELEMENTS_SC}"
if [ ${IS_STUDIO} = true ]; then
	FLEXIV_ELEMENTS_ICON=$(pwd)"/FlexivElementsStudioIcon_512.png" # Flexiv Elements Studio icon file
	echo "Name=Flexiv Elements Studio" >> "${FLEXIV_ELEMENTS_SC}"
else
	FLEXIV_ELEMENTS_ICON=$(pwd)"/flexivicon_512.png"               # Flexiv Elements icon file
	echo "Name=Flexiv Elements" >> "${FLEXIV_ELEMENTS_SC}"
fi
echo "Path=${FLEXIV_ELEMENTS_PATH}" >> "${FLEXIV_ELEMENTS_SC}"
echo "Exec=bash ${FLEXIV_ELEMENTS_EXEC}" >> "${FLEXIV_ELEMENTS_SC}"
echo "Icon=${FLEXIV_ELEMENTS_ICON}" >> "${FLEXIV_ELEMENTS_SC}"
echo "Terminal=true" >> "${FLEXIV_ELEMENTS_SC}"
echo "StartupNotify=false" >> "${FLEXIV_ELEMENTS_SC}"
chmod 744 "${FLEXIV_ELEMENTS_SC}"

# General shortcuts access icon to applications
cp "${FLEXIV_ELEMENTS_SC}" "$HOME"/.local/share/applications/

# Setting Automatic login
Auto_LOGIN="/usr/bin/auto-start-Flexiv-Elements.sh"
sudo rm -rf ${Auto_LOGIN}
sudo touch ${Auto_LOGIN}
sudo chmod 777 ${Auto_LOGIN}
sudo echo "cd \"${FLEXIV_ELEMENTS_PATH}\"" >> ${Auto_LOGIN}
sudo echo "bash run_FlexivElements.sh" >> ${Auto_LOGIN}

echo "[DONE] Create Flexiv ElementsStudio shortcuts... !"
}

# ==============================================================================
# Install dependencies & Delete unused shortcuts
# ==============================================================================
Install_dependencies_and_delete_unused() {
# 1. Install dependencies
common_pkg_deps="libnss3 libxdamage1 qtbase5-dev libxcb-xinerama0 net-tools network-manager iputils-ping \
				freeglut3-dev libatomic1 libgomp1 libasound2"

if [ ${IS_STUDIO} = true ]; then
	sudo apt install -y gocryptfs
fi
sudo apt install -y ${common_pkg_deps}

# Ubuntu 23.04 failed to provide /lib/x86_64-linux-gnu/libglut.so.3 in package, generate manually to local folder
if [ -f "/lib/x86_64-linux-gnu/libglut.so" ]; then
	ln -sf /lib/x86_64-linux-gnu/libglut.so ./lib/libglut.so.3
fi

echo "[DONE] Install dependencies for ${UI_NAME}... !"

sudo apt autoremove -y

# 2. Delete unused shortcuts
# remove G-ROS, IP Config shortcuts
G_ROS_SHORTCUTS=$HOME"/Desktop/G-ROS.desktop"
rm -rf ${G_ROS_SHORTCUTS}
IPCONFIG_SHORTCUTS=$HOME"/Desktop/ConfigIP.desktop"
rm -rf ${IPCONFIG_SHORTCUTS}
# remove system shortcuts
sudo rm -rf /usr/share/applications/G-ROS.desktop
sudo rm -rf /usr/share/applications/ConfigIP.desktop
sudo rm -rf /usr/share/applications/${UI_NAME}.desktop

# rename UIApp name
if [ ${IS_STUDIO} = false ]; then
	if [ -e ./${STUDIO} ]; then
		mv ./${STUDIO} ./${UI_NAME}
		echo "Rename UIApp as ${UI_NAME}"
	fi
else
	if [ -e ./${UI} ]; then
		mv ./${UI} ./${UI_NAME}
		echo "Rename UIApp as ${UI_NAME}"
	fi
fi
}

# ==============================================================================
# Main scripts
# ==============================================================================
# clean up sudo memory
sudo -k

# Command line arguments
usage() {
  echo "Usage: $0 [-e <IS_STUDIO>]"
  echo "    -e : setup as FlexivElements"
  exit 1;
}
while getopts "e" o; do
  case "${o}" in
    e)
      IS_STUDIO=false
	  UI_NAME=${UI}
	  UI_PLATFORM="surface"
      ;;
    *)
      usage
      ;;
  esac
done
shift $((OPTIND-1))

# Get & check password
Get_password_and_check

# Create startup Flexiv Elements script
Create_run_script

# Create Flexiv Elements shortcuts
Create_Flexiv_Elements_shortcuts

# Install dependencies & Delete unused shortcuts
Install_dependencies_and_delete_unused

# ==============================================================================
echo "[DONE] Setup ${UI_NAME}... !"
