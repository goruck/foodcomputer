/**
 * Lambda function for OpenAg Personal Food Computer control and status triggered by Alexa.
 * 
 * For details see https://github.com/goruck.
 *
 * Refactored from original to take advantage of the Alexa Skills Kit SDK for Node.js.
 * 
 * Copyright (c) 2017 Lindo St. Angel
 */

 // 1. Text strings =====================================================================================================
'use strict';
var Alexa = require('alexa-sdk');
var APP_ID = "amzn1.ask.skill.cca03904-f750-41cc-a7cf-beb59fc21820";
var speechOutput = "";
var welcomeOutput = "Please ask the foodcomputer something.";
var welcomeReprompt = "You can say the name of a recipe to start,"
                      +" or ask for the value of a desired or measured parameter,"
                      +" or ask for diagnostics.";

 // 2. Skill Code =======================================================================================================
var handlers = {
    'LaunchRequest': function () {
        this.emit(':ask', welcomeOutput, welcomeReprompt);
    },
    'StartRecipe': function () {
        // Delegate to Alexa for intent and slot confirmations
        var retObj = delegateToAlexa.call(this);

        if (typeof retObj != "undefined") {
            var recipe = this.event.request.intent.slots.recipe.value;
            const method   = "POST",
                  path     = "/_openag/api/0.0.1/service/environments/environment_1/start_recipe",
                  postData = '[\"' + recipe + '\"]';
            if (retObj.confirmationStatus === "CONFIRMED") {
                // Note: arrow functions don't actually bind a this value...
                httpReq (method, path, postData, (resStr) => {
                    var obj = JSON.parse(resStr);
                    if (obj.success) {
                        speechOutput = "started recipe " + recipe;
                    } else {
                        speechOutput = "error, could not start recipe " + recipe;
                    }

                    this.emit(":tell", speechOutput); //...otherwise this will fail
                });
            } else {
                speechOutput = "goodbye";
                this.emit(":tell", speechOutput);
            }
        }
    },
    'GetDesiredParameterValue': function () {
        var parameter = this.event.request.intent.slots.parameter.value;
        var desired = true; // get desired parameters
        getParameterValue.call(this, desired, parameter);
    },
    'GetMeasuredParameterValue': function () {
        var parameter = this.event.request.intent.slots.parameter.value;
        var desired = false; // get measured parameters
        getParameterValue.call(this, desired, parameter);
    },
    'GetDiagnosticInfo': function () {
        const method   = "GET",
              path     = "/_openag/api/0.0.1/topic_data/diagnostics",
              postData = "";

        httpReq (method, path, postData, (resStr) => {
            var obj = JSON.parse(resStr);
            var errMsg = "";

            for (var i = 0; i < obj.status.length ; i++) {
                if (obj.status[i].level !== 0) {
                    errMsg  += (fcDiagToAlexaDiag(obj.status[i].name) + ", ");
                }
            }

            if (errMsg) {
                speechOutput = "The food computer has a problem. " +
                               "The following device or devices are not operating normally. " + errMsg;
            } else {
                speechOutput =  "The food computer is operating normally.";
            }

            this.emit(":tell", speechOutput);
        });
    },
    'AMAZON.HelpIntent': function () {
        this.emit(':ask', welcomeReprompt);
    },
    'AMAZON.CancelIntent': function () {
        speechOutput = "goodbye";
        this.emit(':tell', speechOutput);
    },
    'AMAZON.StopIntent': function () {
        speechOutput = "goodbye";
        this.emit(':tell', speechOutput);
    },
    'SessionEndedRequest': function () {
        speechOutput = "";
        this.emit(':tell', speechOutput);
    },
};

exports.handler = (event, context) => {
    var alexa = Alexa.handler(event, context);
    alexa.appId = APP_ID;
    // To enable string internationalization (i18n) features, set a resources object.
    //alexa.resources = languageStrings;
    alexa.registerHandlers(handlers);
    alexa.execute();
};

//    END of Intent Handlers {} ========================================================================================
// 3. Helper Function  =================================================================================================

/*
 *
 */
function getParameterValue(desired /*true if desired*/, parameter) {
    // Check user request for validity.
    var parameterLowerCase = parameter.toLowerCase(); // no guarantee that Alexa ASR will return value in lower case

    var isValidParameter = checkIfParamIsValid(parameterLowerCase);

    if (isValidParameter) { // parameter valid
        var foodcomputerParam = alexaParamToFoodcomputerParam(parameterLowerCase);
        const method   = "GET",
              path     = "/environmental_data_point/_design/openag/_view/by_variable?group_level=3",
              postData = "";

        httpReq (method, path, postData, (resStr) => {
            var obj = JSON.parse(resStr);
            var paramValue = NaN;
            for (var i = 0; i < obj.rows.length; i++) { // search json obj for the requested information
                if (obj.rows[i].value.variable === foodcomputerParam && 
                    obj.rows[i].value.is_desired === desired) { // found a match
                    paramValue = obj.rows[i].value.value;
                    break;
                }
            }

            var paramValueOut = (paramValue === 0 ? paramValue : paramValue.toFixed(2)); // make it sound nice

            if (!isNaN(paramValueOut)) {
                speechOutput = "the value of " + (desired ? "desired " : "measured ") + parameterLowerCase +
                               " is " + paramValueOut + " " + getParamUnits(foodcomputerParam);
            } else {
                speechOutput = "error, could not get the value of " + (desired ? "desired " : "measured ") +
                               parameterLowerCase;
            }

            this.emit(":tell", speechOutput);
        });
    } else { // parameter not valid
        speechOutput = "error, " + parameterLowerCase + " is not a valid parameter";
        this.emit(":tell", speechOutput);
    }
}

/*
 * Convert Food Computer device names to Alexa friendly names. 
 */
function fcDiagToAlexaDiag(parameter) {
  const diagMap = {"led_blue_1" : "blue lights",
                   "led_white_1" : "white lights",
                   "led_red_1" : "red lights",
                   "chiller_fan_1" : "chiller fan",
                   "chiller_compressor_1" : "chiller compressor",
                   "heater_core_1_1" : "heater core one",
                   "heater_core_2_1" : "heater core two",
                   "ds18b20_1" : "water temperature sensor",
                   "mhz16_1" : "carbon dioxide sensor",
                   "am2315_1" : "air temperature and humidity sensor",
                   "atlas_ph_1" : "ph sensor",
                   "atlas_ec_1" : "water electrical conductivity sensor",
                   "water_level_sensor_high_1" : "high water level sensor",
                   "water_aeration_pump_1" : "water aeration pump",
                   "chiller_pump_1" : "chiller pump",
                   "water_circulation_pump_1" : "water circulation pump",
                   "pump_1_nutrient_a_1" : "pump one, nutrient pump a",
                   "pump_2_nutrient_b_1" : "pump two, nutrient b",
                   "pump_3_ph_up_1" : "pump three, ph up",
                   "pump_4_ph_down_1" : "pump four, ph down",
                   "pump_5_water_1" : "pump five, water",
                   "air_flush_1" : "air flush"};

  var msg = (diagMap.hasOwnProperty(parameter) ? diagMap[parameter] : "unknown device");

  return (msg);
}

/*
 *
 */
function getParamUnits(parameter) {
  const unitsMap = {"air_carbon_dioxide" : "ppm",
                    "air_humidity" : "percent",
                    "air_temperature" : "degrees celsius",
                    "light_illuminance" : "lux", // todo: confirm
                    "light_intensity_blue" : "lux",
                    "light_intensity_red" : "lux",
                    "light_intensity_white" : "lux",
                    "water_electrical_conductivity" : "micro Siemens per centimeter",
                    "water_potential_hydrogen" : " ", // aka pH
                    "water_temperature" : "degrees celsius",
                    "water_level_high" : " "}; // todo: confirm
  return unitsMap[parameter];
}

/*
 * Mapping from Alexa returned parameters to names used in the Foodcomputer database.
 */
function alexaParamToFoodcomputerParam(alexaParam) {
  const paramMap = {"carbon dioxide" : "air_carbon_dioxide",
                    "air carbon dioxide" : "air_carbon_dioxide",
                    "air humidity" : "air_humidity",
                    "humidity" : "air_humidity",
                    "air temperature" : "air_temperature",
                    "temperature" : "air_temperature",
                    "light illuminance" : "light_illuminance",
                    "illuminance" : "light_illuminance",
                    "blue light intensity" : "light_intensity_blue",
                    "blue intensity" : "light_intensity_blue",
                    "red light intensity" : "light_intensity_red",
                    "red intensity" : "light_intensity_red",
                    "white light intensity" : "light_intensity_white",
                    "white intensity" : "light_intensity_white",
                    "water electrical conductivity" : "water_electrical_conductivity",
                    "water conductivity" : "water_electrical_conductivity",
                    "water potential hydrogen" : "water_potential_hydrogen",
                    "water ph level" : "water_potential_hydrogen",
                    "water ph" : "water_potential_hydrogen",
                    "water temperature" : "water_temperature",
                    "water level high" : "water_level_high",
                    "water level hi" : "water_level_high"};
  return paramMap[alexaParam];
}

/*
 *
 */
function checkIfParamIsValid(parameter) {
  const validParameters = ['air carbon dioxide',
                           'carbon dioxide', // alexa won't recognize air
                           'air humidity',
                           'humidity',
                           'air temperature',
                           'temperature',
                           'light illuminance',
                           'illuminance', // alias
                           'blue light intensity',
                           'blue intensity', // alias
                           'red light intensity',
                           'red intensity', // alias
                           'white light intensity',
                           'white intensity', // alias
                           'water electrical conductivity',
                           'water conductivity', // alias
                           'water potential hydrogen', // aka pH
                           'water ph level', // alias
                           'water ph', // alias
                           'water temperature',
                           'water level high',
                           'water level hi']; // alexa returns hi, not high
  var isValidParameter = validParameters.indexOf(parameter) > -1; // true if a valid value was passed
  return isValidParameter;
}

/*
 *
 */
var fs = require('fs'),
    PORT = fs.readFileSync('./port.txt').toString().replace(/\n$/, ''), // Ignore last newline character
    HOST = fs.readFileSync('./host.txt').toString().replace(/\n$/, '');
    //CERT = fs.readFileSync('./client.crt'), // todo: need to enable auth
    //KEY  = fs.readFileSync('./client.key'),
    //CA   = fs.readFileSync('./ca.crt');

var httpReq = (method, path, postData, callback) => {
  var options = {
      hostname: HOST,
      port: PORT,
      path: path,
      method: method,
      headers: {
        "Content-Type": "application/json",
        "Content-Length": postData.length
      }
    };

  var http = require("http");
  
  var req = http.request(options, (res) => {
    var resStr = "";

    res.on("data", (chunk) => {
        //console.log('chunk: ' + chunk);
        resStr += chunk;
    });

    res.on("end", () => {
      //console.log('STATUS: ' + res.statusCode);
      //console.log('HEADERS: ' + JSON.stringify(res.headers));
      callback(resStr);
    });
  });

  req.write(postData);

  req.end();

  req.on('error', (e) => {
    console.log('problem with request: ' + e.message);
  });
}

/*
 *
 */
function delegateToAlexa() {
    //console.log("in delegateToAlexa");
    //console.log("current dialogState: "+ this.event.request.dialogState);

    if (this.event.request.dialogState === "STARTED") {
        //console.log("in dialog state STARTED");
        var updatedIntent = this.event.request.intent;
        //optionally pre-fill slots: update the intent object with slot values for which
        //you have defaults, then return Dialog.Delegate with this updated intent
        // in the updatedIntent property
        this.emit(":delegate", updatedIntent);
    } else if (this.event.request.dialogState !== "COMPLETED") {
        //console.log("in dialog state COMPLETED");
        // Return a Dialog.Delegate directive with no updatedIntent property
        this.emit(":delegate");
    } else {
        //console.log("dialog finished");
        //console.log("returning: "+ JSON.stringify(this.event.request.intent));
        // Dialog is now complete and all required slots should be filled,
        // so call your normal intent handler.
        return this.event.request.intent;
    }
}

/*
 *
 */
function randomPhrase(array) {
    // the argument is an array [] of words or phrases
    var i = 0;
    i = Math.floor(Math.random() * array.length);
    return(array[i]);
}

/*
 *
 */
function isSlotValid(request, slotName){
    var slot = request.intent.slots[slotName];
    //console.log("request = "+JSON.stringify(request)); //uncomment if you want to see the request
    var slotValue;

    //if we have a slot, get the text and store it into speechOutput
    if (slot && slot.value) {
         //we have a value in the slot
         slotValue = slot.value.toLowerCase();
        return slotValue;
     } else {
         //we didn't get a value in the slot.
         return false;
     }
}
