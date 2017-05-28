*This is still very much a work in progress.*

# Alexa OpenAg Personal Food Computer
This describes how to use Amazon's Alexa to create a voice user interface to the OpenAg Personal Food Computer (PFC). I was very fortunate to meet the leaders of Fenome and got the opportunity to build my own OpenAg PFC which is now in use at my home. I thought I'd give back to Fenome and the OpenAg community by integrating Amazon's Alexa with the PFC. I find the PFC an amazing device and the OpenAg initiative inspiring! I hope people find the PFC even more useful with Alexa.

Here are some examples of what you can do.

Example User Request | Note
---------------------|------------------
Get the value of a desired parameter
“Alexa, ask Food Computer for desired air carbon dioxide” | Returns desired air CO2 in ppm
"Alexa, ask Food Computer what the desired air humidity is" | Many alt ways of asking for a parameter are supported
“Alexa, ask Food Computer for desired {parameter}” | See below for complete list of parameters supported
Get the value of a measured parameter
“Alexa, ask Food Computer for measured air carbon dioxide” | Returns measured air CO2 in ppm
“Alexa, ask Food Computer for air carbon dioxide” | Omitting measured or desired defaults to measured
“Alexa, ask Food Computer for water potential hydrogen” | aka pH
“Alexa, ask Food Computer for water pH level” | Alias for potential hydrogen
“Alexa, ask Food Computer for measured {parameter}” | Measured is an optional keyword
Start a recipe
"Alexa, ask Food Computer for recipe lettuce" | Will start the recipe called "lettuce"
"Alexa, ask Food Computer to start recipe" lettuce | alt way of starting recipe "lettuce"
Get Food Computer diagnostics information
"Alexa, ask Food Computer for diagnostics" | Returns the health of the system
"Alexa, ask Food Computer how its feeling" | alt way of asking for diags
Set the value of a desired parameter
"Alexa ask Food Computer to set air temperature to 30 degrees" | *in progress*
Get plant health and any issues that need attention
"Alexa", ask Food Computer how my plants are | *in progress*

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

This Alexa skill is not yet published. If you want to try it out right now, you need to set up an Amazon applications developer account and an Amazon Web Services account. The JSON in this repo's ask directory can be used in your dev account to create the Alexa skill and the node.js code in the lambda directory will need to run in your own lambda instance. Eventually I will publish the skill (pending approval by Amazon) so anyone with a Food Computer can use it without needing to clone this repo. 

# Requirements and System Architecture
*coming soon*

# Personal Food Computer Preparation
*coming soon*

# Alexa Skills Development
*coming soon*

# AWS Lamba Function Development
*coming soon*

# Computer Vision Development
*coming soon*

# Development and Test Environment
*coming soon*

# Licensing
Everything here is licensed under the [MIT license]https://choosealicense.com/licenses/mit/.

# Contact Information
For questions or comments about this project please contact the author goruck (Lindo St. Angel) at {lindostangel} AT {gmail} DOT {com}.

# Appendix
*coming soon*
