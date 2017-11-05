*The Food Computer Alexa skill is now in beta test! Please send me an e-mail to be a part of the beta and you'll be able to use the skill to interact with my food computer on your Echo devices. Any feedback is welcome but please be aware this skill and the Personal Food Computer project are still in development.*

# Alexa OpenAg Personal Food Computer
This describes how to use Amazon's Alexa to create a voice user interface to the OpenAg Personal Food Computer (PFC). I was very fortunate to meet the leaders of Fenome and got the opportunity to build my own OpenAg PFC which is now in use at my home. I thought I'd give back to Fenome and the OpenAg community by integrating Amazon's Alexa with the PFC. I find the PFC an amazing device and the OpenAg initiative inspiring! I hope people find the PFC even more useful with Alexa.

This Alexa skill is in beta test. If you want to try it out right now you can either be included in my beta test and interact with my PFC via my skill on your own Echo device or fork my code and run it as your own skill if you have your own PFC.

# Requirements and System Architecture
I had the following goals for this project which lead to certain requirements.

1. **Learn about the OpenAg initiative and and how might people benefit from this technology.**
2. **Learn how to develop an Alexa skill for Echo devices with a display and in particular skills for the Internet of Things.** This lead to the requirement to use an external plot creation service (plot.ly), for example. Note that I did consider using the AWS IoT service for this project but decided against it since the PFC is a relatively complex device and in particular has an integrated database. 
3. **Learn how to integrate a voice UI with the PFC that gave a great user experience.** This required me to not only think through the user interaction but also help fix stability bugs in the OpenAg Brain software and to further develop its API to facilitate the integration with Alexa. I also had to pay attention how fast Alexa responded to requests with five seconds or less as the requirement. This drove some decisions such as how to access the PFC's CouchDB database, in particular the use of stale views. Latency optimization is a work in progress. 
4. **Learn about ROS which is extensively used in the OpenAg Brain project.**
5. **Create an end-to-end framework for Alexa Skills development that can be used for other similar IoT applications.** This lead to the requirement to use standard services and components where possible and provide clear documentation. 
6. **Ensure that the end-to-end service is secure.** This required me to enable HTTPS on CouchDB and in the JavaScript code that handles the Alexa intent.
7. **Learn about how computer vision can be used in growing plants optimally.** This lead to the use of OpenAg CV and the integration of it with OpenAg Brain. 

These goals and requirements eventual lead to the following system architecture.

![Alt text](/img/pfc-blk-dia.jpg?raw=true "Based on Gordon Brander's architecture diagram.")

# Usage

Example User Request | Note
---------------------|------------------
**Get information about the recipe being used**
"Alexa, ask Food Computer for recipe information" | Returns current recipe name and when started
**Show graph of a parameter (if Echo device has a display)** | Latency may be high - be patient.
"Alexa, ask Food Computer to graph air carbon dioxide" | Default is to graph over last 24 hours
"Alexa, ask Food Computer to graph {parameter}" | See below for list of parameters supported
**Show camera image (if Echo device has a display)**
"Alexa, ask Food Computer to show top camera" | Displays latest image from aerial camera
"Alexa, ask Food Computer to show side camera" | Latest image from side camera
"Alexa, ask Food Computer to show top camera {time} ago" | Top camera image from specified past time
**Show plant size and number of leaves**
"Alexa, ask Food Computer to show measurement view" | uses openag_cv
**Get plant health and any issues that need attention**
"Alexa, ask Food Computer how my plants are" | *in progress*
**Get Food Computer diagnostics information**
"Alexa, ask Food Computer for diagnostics" | Returns the health of the system
"Alexa, ask Food Computer how its feeling" | alt way of asking for diags
**Set the value of a desired parameter**
"Alexa ask Food Computer to set air temperature to 30 degrees" | *in progress*
**Get the value of a measured parameter (and desired parameter if set)**
“Alexa, ask Food Computer for air carbon dioxide” | Returns measured air CO2 in ppm
“Alexa, ask Food Computer for air temperature” | Returns measured air temp and desired temp if set by recipe
“Alexa, ask Food Computer for water potential hydrogen” | aka pH
“Alexa, ask Food Computer for water pH level” | Alias for potential hydrogen
“Alexa, ask Food Computer for desired {parameter}” | See below for list of parameters supported
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

# Alexa Skills, AWS Lambda and S3
The JSON in this repo's ask directory can be used in your dev account to create the Alexa skill and the JavaScript in the lambda directory will need to run in your own lambda instance (using node.js as the runtime) as it serves as the intent handler for the skill.

To write your own version of this skill you need to set up an Amazon applications developer account and an Amazon Web Services account. See this excellent [tutorial](https://github.com/alexa/alexa-cookbook/tree/master/handling-responses/dialog-directive-delegate#title) for an example of how to do this and get started writing Alexa skills. I used the [Alexa Skills Kit SDK for Node.js](https://www.npmjs.com/package/alexa-sdk) to develop this application.

I'm using AWS S3 to temporary store images from the PFC's cameras and the plots of PFC variable data over time from Plotly service. This is required since the Alexa service needs a URL to an image to be displayed. I needed to setup a S3 bucket for this purpose and give the lambda service permission to access it, see [this](http://docs.aws.amazon.com/lambda/latest/dg/with-s3.html) for how to do that. 

# OpenAg Brain
I've made modifications to the openag_brain source to facilitate integration with Alexa and although some of these changes have been integrated into the official openag_brain repo, to get the latest you should use my "cv" [fork](https://github.com/goruck/openag_brain/tree/cv) of openag_brain.

# Computer Vision
I'm using openag_cv for CV development. I've also made modifications to the openag_cv source to get it to work with an integrated openag_brain configuration, so you need to use my [fork](https://github.com/goruck/openag_cv) of openag_cv.

# Plotly
I'm using a fantastic plotting service called [Plotly](https://plot.ly/) to generate plots from the PFC's database upon a user request to Alexa. I needed to setup a free Plotly account to use it. The lambda code sends the PFC data to the Plotly service which returns a png image. The latency of the service tends to be a second or two but I've seen it as long as five seconds. Although this is not the bottleneck to show a PFC variable plot to the user it can be optimized by running the Plotly code in the lambda instance. This would required it to be compiled as a node package and uploaded with the rest of my handler code to lambda. 

# Licensing
Everything here is licensed under the [MIT license](https://choosealicense.com/licenses/mit/).

# Contact Information
For questions or comments about this project please contact the author goruck (Lindo St. Angel) at {lindostangel} AT {gmail} DOT {com}.

# Appendix

## Installing OpenAg_Brain (openag_brain) on the Food Computer's Raspberry Pi
1. Download [Raspbian Jessie Lite](https://www.raspberrypi.org/downloads/raspbian/).
2. Download [Etcher](https://etcher.io/).
3. Use Etcher to write Raspbian image to a micro SD card. I used a 64GB card.
4. Insert SD card in Raspberry Pi, boot to console. Make sure the Raspberry Pi has an Ethernet connection. 
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
$ cd ~/catkin_ws/src/openag_brain
$ ./scripts/firmware -t upload
```
*Note: you may have to remove and reinsert the Arduino USB cable from the Raspberry pi after the last command.*

15. Test that openag brain works:
```bash
$ rosrun openag_brain main personal_food_computer_v2.launch
```

## Installing the OpenAg User Interface (openag_ui) on the Food Computer's Raspberry Pi
1. [Install NodeJs and NPM for openag_ui](https://tecadmin.net/install-latest-nodejs-npm-on-debian/):
```bash
$ sudo apt-get install curl python-software-properties
$ curl -sL https://deb.nodesource.com/setup_7.x | sudo bash -
$ sudo apt-get install nodejs
```
2. [Clone openag_ui source code](https://github.com/OpenAgInitiative/openag_ui):
```bash
$ git clone https://github.com/OpenAgInitiative/openag_ui ~/
```
3. [Build and Deploy the UI](https://github.com/OpenAgInitiative/openag_ui):
```bash
$ cd openag_ui
$ npm install
$ npm run couchapp_deploy --app_db_url="http://localhost:5984/app"
```
4. Test that the UI works Ok:
Open your browser to http://localhost:5984/app/_design/app/_rewrite.

## Installing OpenAg Computer Vision (openag_cv) on the Food Computer's Raspberry Pi
Clone my fork of openag_brain source code:
```bash
git clone https://github.com/goruck/openag_cv.git ~/catkin_ws/src/openag_cv
```

## Misc Setup and Configuration
1. [Setup wifi on the Raspberry Pi](https://www.raspberrypi.org/documentation/configuration/wireless/wireless-cli.md) (optional).

2. Modify rsyslog configuration file to prevent rsyslog messages flooding the logs which may eventually cause a system wide crash. See [this](https://www.raspberrypi.org/forums/viewtopic.php?f=91&t=122601) for details. You may have this problem if the Raspberry Pi occasionally crashes and the only way to recover is power cycling it. Also you may see messages like the following in /var/log/messages:

```text
Sep  3 11:27:12 raspberrypi rsyslogd-2007: action 'action 17' suspended, next retry is Sun Sep  3 11:28:42 2017 [try http://www.rsyslog.com/e/2007 ]
Sep  3 11:28:42 raspberrypi rsyslogd-2007: action 'action 17' suspended, next retry is Sun Sep  3 11:30:12 2017 [try http://www.rsyslog.com/e/2007 ]
\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\0
```

Note \00... is binary junk after the actual crash and the action 17 message before that is repeated many times.

To fix, comment out the last 4 lines of /etc/rsyslog.conf file like this:
```text
#daemon.*;mail.*;\
#       news.err;\
#       *.=debug;*.=info;\
#       *.=notice;*.=warn       |/dev/xconsole
```

3. [Setup email](http://www.sbprojects.com/projects/raspberrypi/exim4.php) on the Raspberry Pi (optional).

4. Enable the watchdog timer. The PFC and the openag brain in particular is a complex system that is still in development and as such the Raspberry Pi software can occasionally freeze up. To ensure the health of the plants the Raspberry Pi's watchdog timer should be enabled which will automatically reboot the Raspberry Pi in the event of a software lockup. You also need to enable openag_brain to run as a service that automatically starts after the Raspberry Pi boots. The instructions for doing so can be found [here](http://forkgeeks.com/enabling-watchdog-on-raspberry-pi/). Note that you should test the watchdog by issuing a so-called forkbomb as follows.

```bash
$ swapoff -a
$ forkbomb(){ forkbomb | forkbomb & }; forkbomb
```

If you have email setup the watchdog will send you and email when it has timed out and triggered a reboot. 

## Securing the PFC
The Alexa skill's code runs in an AWS Lambda instance which communicates with the PFC over the Internet via CouchDB REST APIs and openag_brain REST APIs proxied by CouchDB. For security purposes, these APIs need to be authenticated and encrypted via TLS/SSL and the CouchDB "admin party" needs to be ended. Here are the steps to do this.

1. Add an admin account to CouchDB, ending the admin party. The example (taken from [this](http://guide.couchdb.org/draft/security.html)) below assumes the admin's username is name with password secret.
```bash
$ curl -X PUT $HOST/_config/admins/name -d '"secret"'
```

2. Note that after the admin is created, couchdb needs to be initialized as follows. Note that the patch [below](https://github.com/goruck/foodcomputer#list-of-modifications-done-to-openag_brain-for-integration-with-alexa) needs to be applied so that couchdb accepts the admin name and password. 

```bash
$ # (if required) detach from the local server first
$ openag db deinit
$ openag db init --db_url http://name:passwd@localhost:5984
```

3. Generate server and client certificates and key pairs via OpenSSL. Mutual authentication and self-signed certs will be used. Production environments should use certs signed by an actual CA. This assumes OpenSSL is installed on the Raspberry Pi, it should be by default. See [CouchDB HTTP Server](http://docs.couchdb.org/en/2.0.0/config/http.html) for additional details.
```bash
$ # login as root to run all these commands
$ su -
$ mkdir /etc/couchdb/cert
$ cd /etc/couchdb/cert
$ # Generate ca private key
$ openssl genrsa -out ca.key 4096
$ # Create self-signed ca cert, COMMON_NAME="My CA"
$ openssl req -new -x509 -days 365 -key ca.key -out ca.crt -sha256
$ # Create client private key
$ openssl genrsa -out client.key 2048
$ # Create client cert signing request, COMMON_NAME="Client 1"
$ openssl req -new -key client.key -out client.csr -sha256
$ # Create self-signed client cert
$ openssl x509 -req -days 365 -in client.csr -CA ca.crt \
> -CAkey ca.key -set_serial 01 -out client.crt -sha256
$ # Create server private key
$ openssl genrsa -out server.key 2048
$ # Create server cert signing request, COMMON_NAME="localhost"
$ openssl req -new -key server.key -out server.csr -sha256
$ # Create signed server cert, where "key.ext" contains "subjectAltName = IP:xxx.xxx.xxx.xxx"
$ # IP is the external IP address of your PFC.
$ openssl x509 -req -days 365 -in server.csr -CA ca.crt \
$ > -CAkey ca.key -set_serial 02 -out server.crt -sha256 -extfile key.ext
$ # Copy client key pair and CA certificate to Lambda source code directory.
$ # These will need to be uploaded to AWS via a zip file which also includes the Lambda Node.js code.
$ cp client.crt path-to-lambda-code/lambda
$ cp client.key path-to-lambda-code/lambda
$ cp ca.crt path-to-lambda-code/lambda
$ # Set file ownership and permissions appropriately
$ chmod 600 *
$ chown couchdb:couchdb *
$ # logout as root
$ exit
```

4. Edit CouchDB’s configuration, by editing your local.ini. Change the following sections in the local.ini file (should be in /etc/couchdb).

```text
[daemons]
httpsd = {couch_httpd, start_link, [https]} # Enable the HTTPS daemon

[ssl]
cert_file = /etc/couchdb/cert/server.crt
key_file = /etc/couchdb/cert/server.key
cacert_file = /etc/couchdb/cert/ca.crt
verify_ssl_certificates = true # Set to true to validate peer (client) certificates
fail_if_no_peer_cert = true # Set to true to terminate the TLS/SSL handshake if the client does not send a certificate
```

Note: CouchDB v1.6.0 (standard with the PFC) does not support the fail_if_no_peer_cert directive but v2.0.0 does. Therefore to ensure robust mutual authentication v2.0.0 should be used with the PFC. 

5. Restart CouchDB so that the modified local.ini file will take effect.
```bash
$ sudo service couchdb stop
$ sudo service couchdb start
```

6. Test using the external IP address and port number of the PFC. 
```bash
$ curl --cacert ca.crt --key client.key --cert client.crt https://external-ip-addr:external-port-num/
{"couchdb":"Welcome","uuid":"1d737ecdddede0ece99992f4e8dea743","version":"1.6.0","vendor":{"version":"8.3","name":"Debian"}}
```

## List of Modifications done to Openag_Brain for integration with Alexa (including stablity fixes)

1. https://github.com/OpenAgInitiative/openag_brain/pull/336
2. https://github.com/OpenAgInitiative/openag_brain/pull/335
3. https://github.com/OpenAgInitiative/openag_brain/pull/334
4. https://github.com/OpenAgInitiative/openag_brain/pull/317
5. https://github.com/OpenAgInitiative/openag_brain/pull/294
6. https://github.com/OpenAgInitiative/openag_brain/pull/262
7. https://github.com/OpenAgInitiative/openag_brain/pull/339
