*This is still very much a work in progress*

# Alexa OpenAg Personal Food Computer
This describes how to use Amazon's Alexa to create a voice user interface to the OpenAg Personal Food Computer (PFC). I was very fortunate to meet the leaders of Fenome and got the opportunity to build my own OpenAg PFC which is now in use at my home. I thought I'd give back to Fenome and the OpenAg community by integrating Amazon's Alexa with the PFC. I find the PFC an amazing device and the OpenAg initative inspiring! I hope people find the PFC even more useful with Alexa.

Here are some examples of what you can do at the present.

Example User Request | Note
---------------------|------------------
“Alexa, ask Food Computer for desired air carbon dioxide” | returns desired air CO2 in ppm
“Alexa, ask Food Computer for measured air carbon dioxide” | returns measured air CO2 in ppm
“Alexa, ask Food Computer for air carbon dioxide” | omitting measured or desired defaults to measured
“Alexa, ask Food Computer for water potential hydrogen” | aka pH
“Alexa, ask Food Computer for water pH level” | alias for potential hydrogen
“Alexa, ask Food Computer for desired {parameter}” | see below for complete list of parameters supported
“Alexa, ask Food Computer for measured {parameter}” | measured is an optional keyword
"Alexa, ask Food Computer what the desired air humidity is" | many alt ways of asking for a parameter are supported
"Alexa, ask Food Computer for a million dollars" | will return an error to the user
"Alexa, ask Food Computer for recipe lettuce | will start the recipe called "lettuce"
"Alexa, ask Food Computer to start recipe lettuce | alt way of starting recipe "lettuce"

Here are the list of currently supported parameters accessable by Alexa.

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
TBA

# Personal Food Computer Preparation
TBA

# Alexa Skills Development
TBA

# AWS Lamba Function Development
TBA

# Computer Vision Development
TBA

# Development and Test Environment
TBA

# Licensing
TBA

# Contact Information
For questions or comments about this project please contact the author goruck (Lindo St. Angel) at {lindostangel} AT {gmail} DOT {com}.

# Appendix
TBA
