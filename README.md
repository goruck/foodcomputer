*This is still very much a work in progress.*

# Alexa OpenAg Personal Food Computer
This describes how to use Amazon's Alexa to create a voice user interface to the OpenAg Personal Food Computer (PFC). I was very fortunate to meet the leaders of Fenome and got the opportunity to build my own OpenAg PFC which is now in use at my home. I thought I'd give back to Fenome and the OpenAg community by integrating Amazon's Alexa with the PFC. I find the PFC an amazing device and the OpenAg initiative inspiring! I hope people find the PFC even more useful with Alexa.

This Alexa skill is not yet published. If you want to try it out right now you need to set up an Amazon applications developer account and an Amazon Web Services account. See this excellent [tutorial](https://github.com/alexa/alexa-cookbook/tree/master/handling-responses/dialog-directive-delegate#title) for an example of how to do this and get started writing Alexa skills. I used the [Alexa Skills Kit SDK for Node.js](https://www.npmjs.com/package/alexa-sdk) to develop this application. 

The JSON in this repo's ask directory can be used in your dev account to create the Alexa skill and the node.js code in the lambda directory will need to run in your own lambda instance. I've made some modifications to the openag_brain source to facilitate integration with Alexa and although some of these changes have been integrated into the official openag_brain repo, to get the latest you should use my fork of openag_brain. 

Eventually I will publish the skill (pending approval by Amazon) so anyone with a Food Computer can use it without needing to clone this repo. 

Here are some examples of what you can do.

Example User Request | Note
---------------------|------------------
**Get plant health and any issues that need attention**
"Alexa, ask Food Computer how my plants are" | *in progress*
**Get Food Computer diagnostics information**
"Alexa, ask Food Computer for diagnostics" | Returns the health of the system
"Alexa, ask Food Computer how its feeling" | alt way of asking for diags
**Get the value of a desired parameter**
“Alexa, ask Food Computer for desired air carbon dioxide” | Returns desired air CO2 in ppm
"Alexa, ask Food Computer what the desired air humidity is" | Many alt ways of asking for a parameter are supported
“Alexa, ask Food Computer for desired {parameter}” | See below for complete list of parameters supported
**Set the value of a desired parameter**
"Alexa ask Food Computer to set air temperature to 30 degrees" | *in progress*
**Get the value of a measured parameter**
“Alexa, ask Food Computer for measured air carbon dioxide” | Returns measured air CO2 in ppm
“Alexa, ask Food Computer for air carbon dioxide” | Omitting measured or desired defaults to measured
“Alexa, ask Food Computer for water potential hydrogen” | aka pH
“Alexa, ask Food Computer for water pH level” | Alias for potential hydrogen
“Alexa, ask Food Computer for measured {parameter}” | Measured is an optional keyword
**Start a recipe**
"Alexa, ask Food Computer for recipe lettuce" | Will start the recipe called "lettuce"
"Alexa, ask Food Computer to start recipe" lettuce | alt way of starting recipe "lettuce"

Here is a list of currently supported Food Computer parameters accessible by Alexa.

Spoken Parameter | Food Computer Database Parameter
----------|-----------------------------------------
"carbon dioxide" | "air_carbon_dioxide"
"air carbon dioxide" | "air_carbon_dioxide"
"air humidity" | "air_humidity"
"humidity" | "air_humidity"
"air temperature" | "air_temperature"
"temperature" | "air_temperature"
"light illuminance" | "light_illuminance"
"illuminance" | "light_illuminance"
"blue light intensity" | "light_intensity_blue"
"blue intensity" | "light_intensity_blue"
"red light intensity" | "light_intensity_red"
"red intensity" | "light_intensity_red"
"white light intensity" | "light_intensity_white"
"white intensity" | "light_intensity_white"
"water electrical conductivity" | "water_electrical_conductivity"
"water conductivity" | "water_electrical_conductivity"
"water potential hydrogen" | "water_potential_hydrogen"
"water ph level" | "water_potential_hydrogen"
"water ph" | "water_potential_hydrogen"
"water temperature" | "water_temperature"
"water level high" | "water_level_high"

# Requirements and System Architecture
*coming soon*

# Personal Food Computer Preparation

## Installing Openag_brain on the Food Computer's Raspberry Pi
1. Download [Raspbian Jessie Lite](https://www.raspberrypi.org/downloads/raspbian/).
2. Download [Etcher](https://etcher.io/).
3. Use Etcher to write Raspian image to a micro SD card. I used a 64GB card.
4. Insert SD card in Raspberry Pi, boot to console. Make sure the Raspberry Pi has an ethernet connection. 
5. Change default password:
```bash
$ echo "password" | passwd --stdin pi
```
6. Update Raspbian:
```bash
$ sudo apt-get update && sudo apt-get upgrade
```
7. Start sshd at reboot to allow headless operation via ssh:
```bash
$ sudo systemctl start ssh.service
$ sudo systemctl enable ssh.service
```
8. Add US English as a locale (optional):
```bash
$ sudo dpkg-reconfigure locales
```
9. Set time zone:
```bash
$ sudo tzconfig
```
10. Install git:
```bash
$ sudo apt-get install git
```
11. Clone my fork of openag_brain source code, similar to [this](https://wiki.openag.media.mit.edu/openag_brain/installing/installing_globally).
```bash
$ git clone https://github.com/goruck/openag_brain.git ~/catkin_ws/src/openag_brain
```
12. [Increase swap space](https://wiki.openag.media.mit.edu/openag_brain/installing/installing_globally).
13. [Install and compile openag_brain](https://wiki.openag.media.mit.edu/openag_brain/installing/installing_globally):
```bash
$ cd ~/catkin_ws/src/openag_brain
$ ./scripts/install_dev
```
14. Build firmware and flash to Arduino:
```bash
$ source /opt/ros/indigo/setup.bash
$ cd /opt/ros/indigo/share/openag_brain/launch
$ rosrun openag_brain firmware -t upload ./personal_food_computer_v2.yaml -p ros
```
*Note: you may have to remove and reinsert the Arduino USB cable from the Raspberry pi after the last command.*

15. Test that openag brain works:
```bash
$ rosrun open_brain main personal_food_computer_v2.launch
```
16. [Install NodeJs and NPM for openag_ui](https://tecadmin.net/install-latest-nodejs-npm-on-debian/):
```bash
$ sudo apt-get install curl python-software-properties
$ curl -sL https://deb.nodesource.com/setup_7.x | sudo bash -
$ sudo apt-get install nodejs
```
17. [Clone openag_ui source code](https://github.com/OpenAgInitiative/openag_ui):
```bash
$ git clone https://github.com/OpenAgInitiative/openag_ui
```
18. [Build and Deploy the UI](https://github.com/OpenAgInitiative/openag_ui):
```bash
$ cd openag_ui
$ npm install
$ npm run couchapp_deploy --app_db_url="http://localhost:5984/app"
```
19. Test that the UI works Ok: Open your browser to http://${IP_OF_FOOD_COMPUTER}:5984/app/_design/app/_rewrite.
20. [Setup wifi on the Raspberry Pi](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) (optional).

## List of Modifications done to Openag_Brain for integration with Alexa

1. Added get_topic_data() in openag_brain/nodes/api.py

2. Fixed openag_brain issue #252

3. Added couchdb authentication support in openag/src/openag_python/openag/cli/db/\__init__.py.
Note: openag_python is being deprecated, so this change will need to be applied to https://github.com/OpenAgInitiative/openag_brain/blob/develop/src/openag_lib/db_bootstrap/db_init.py. 

# Alexa Skills Development
*coming soon*

# AWS Lamba Function Development
*coming soon*

# Computer Vision Development
*coming soon*

# Development and Test Environment
*coming soon*

# Licensing
Everything here is licensed under the [MIT license](https://choosealicense.com/licenses/mit/).

# Contact Information
For questions or comments about this project please contact the author goruck (Lindo St. Angel) at {lindostangel} AT {gmail} DOT {com}.

# Appendix
*coming soon*
