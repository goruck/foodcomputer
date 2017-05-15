/**
 * Lambda function for OpenAg Personal Food Computer control and status triggred by Alexa.
 * 
 * For details see https://github.com/goruck.
 * 
 * Copyright (c) 2017 Lindo St. Angel
 */

// Route the incoming request based on type (LaunchRequest, IntentRequest,
// etc.) The JSON body of the request is provided in the event parameter.
exports.handler = function (event, context) {
    try {
        console.log("event.session.application.applicationId=" + event.session.application.applicationId);

        /**
         * Prevents someone else from configuring a skill that sends requests to this function.
         */
        if (event.session.application.applicationId !== "amzn1.ask.skill.cca03904-f750-41cc-a7cf-beb59fc21820") {
             context.fail("Invalid Application ID");
         }

        if (event.session.new) {
            onSessionStarted({requestId: event.request.requestId}, event.session);
        }

        if (event.request.type === "LaunchRequest") {
            onLaunch(event.request,
                     event.session,
                     function callback(sessionAttributes, speechletResponse) {
                        context.succeed(buildResponse(sessionAttributes, speechletResponse));
                     });
        }  else if (event.request.type === "IntentRequest") {
            onIntent(event.request,
                     event.session,
                     function callback(sessionAttributes, speechletResponse) {
                         context.succeed(buildResponse(sessionAttributes, speechletResponse));
                     });
        } else if (event.request.type === "SessionEndedRequest") {
            onSessionEnded(event.request, event.session);
            context.succeed();
        }
    } catch (e) {
        context.fail("Exception: " + e);
    }
};

/**
 * Called when the session starts.
 */
function onSessionStarted(sessionStartedRequest, session) {
    console.log("onSessionStarted requestId=" + sessionStartedRequest.requestId +
                ", sessionId=" + session.sessionId);
}

/**
 * Called when the user launches the skill without specifying what they want.
 */
function onLaunch(launchRequest, session, callback) {
    console.log("onLaunch requestId=" + launchRequest.requestId +
                ", sessionId=" + session.sessionId);

    // Dispatch to skill's launch.
    getWelcomeResponse(callback);
}

/**
 * Called when the user specifies an intent for this skill.
 */
function onIntent(intentRequest, session, callback) {
    console.log("onIntent requestId=" + intentRequest.requestId +
                ", sessionId=" + session.sessionId +
                ", intentName=" + intentRequest.intent.name);

    var intent = intentRequest.intent,
        intentName = intentRequest.intent.name;

    // Dispatch to skill's intent handlers
    if ("StartRecipe" === intentName) {
        startRecipe(intent, session, callback);
    } else if ("StopRecipe" === intentName) {
        stopRecipe(intent, session, callback);
    } else if ("GetDesiredParameterValue" === intentName) {
        getParameterValue(true, intent, session, callback);
    } else if ("GetMeasuredParameterValue" === intentName) {
        getParameterValue(false, intent, session, callback);
    } else if ("AMAZON.HelpIntent" === intentName) {
        getWelcomeResponse(callback);
    } else if ("AMAZON.StopIntent" === intentName) {
        stopSession(callback);
    } else if ("AMAZON.CancelIntent" === intentName) {
        stopSession(callback);
    } else {
        throw "Invalid intent";
    }
}

/**
 * Called when the user ends the session.
 * Is not called when the skill returns shouldEndSession=true.
 */
function onSessionEnded(sessionEndedRequest, session) {
    console.log("onSessionEnded requestId=" + sessionEndedRequest.requestId +
            ", sessionId=" + session.sessionId);
    // Add cleanup logic here
}

// --------------- Functions that control the skill's behavior -----------------------

function stopSession(callback) {
    var sessionAttributes = {};
    var cardTitle = "Goodbye";
    var speechOutput = "goodbye";
    var shouldEndSession = true;
    var repromptText = "";

    callback(sessionAttributes,
             buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

function getWelcomeResponse(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var cardTitle = "Welcome";
    var speechOutput = "please ask the food computer something";
    // If the user either does not reply to the welcome message, they will be prompted again.
    var repromptText = "please ask the food computer something, for example to start a recipe"
                       + " or ask for the value of a desired or measured parameter";
    var shouldEndSession = false;

    callback(sessionAttributes,
             buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

/*
 * Returns the most recent desired setpoint (desired === true) or measured value (desired === false)
 * of a parameter from the foodcomputer's database.
 */
function getParameterValue(desired /*true if desired*/, intent, session, callback) {
  var cardTitle = intent.name;
  var parameter = intent.slots.parameter.value;
  var sessionAttributes = {};
  var repromptText = "";
  var shouldEndSession = true;
  var speechOutput = "";

  // Check user request for validity.
  var parameterLowerCase = parameter.toLowerCase(); // no guarantee that Alexa ASR will return value in lower case
  var isValidParameter = checkIfParamIsValid(parameterLowerCase);

  if (isValidParameter) { // parameter valid
    var foodcomputerParam = alexaParamToFoodcomputerParam(parameterLowerCase);
    //console.log("FCParam: " + foodcomputerParam);

    const method   = "GET",
          path     = "/environmental_data_point/_design/openag/_view/by_variable?group_level=3",
          postData = "";

    httpReq (method, path, postData, function (obj) {
      var paramValue = NaN;
      for (var i = 0; i < obj.rows.length; i++) { // search json obj for the requested information
        if (obj.rows[i].value.variable === foodcomputerParam && 
            obj.rows[i].value.is_desired === desired) { // found a match
          paramValue = obj.rows[i].value.value;
          break;
        }
      }
      //console.log("value: " + paramValue);

      var paramValueOut = (paramValue === 0 ? paramValue : paramValue.toFixed(2)); // make it sound nice

      if (!isNaN(paramValueOut)) {
        speechOutput = "the value of " + (desired ? "desired " : "measured ") + parameterLowerCase +
                       " is " + paramValueOut + " " + getParamUnits(foodcomputerParam);
      } else {
        speechOutput = "error, could not get the value of " + (desired ? "desired " : "measured ") +
                       parameterLowerCase;
      }

      callback(sessionAttributes,
               buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
    });
  } else { // parameter not valid
    speechOutput = "error, " + parameterLowerCase + " is not a valid parameter";
    callback(sessionAttributes,
             buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
  }
}

/*
 * Starts a recipe.
 */
function startRecipe(intent, session, callback) {
  var cardTitle = intent.name;
  var recipe = intent.slots.recipe.value;
  var sessionAttributes = {};
  var repromptText = "";
  var shouldEndSession = true;
  var speechOutput = "";

  const method   = "POST",
        path     = "/_openag/api/0.0.1/service/environments/environment_1/start_recipe",
        postData = '[\"' + recipe + '\"]';

  //console.log("postData: " + postData);

  httpReq (method, path, postData, function (obj) {
    if (obj.success) {
      speechOutput = "started recipe " + recipe;
    } else {
      speechOutput = "error, could not start recipe " + recipe;
    }
    callback(sessionAttributes,
             buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
  });
}

// --------------- global variables and commonly used functions -----------------------
/*
 *
 */
var fs = require('fs'),
    PORT = fs.readFileSync('./port.txt').toString().replace(/\n$/, ''), // Ignore last newline character
    HOST = fs.readFileSync('./host.txt').toString().replace(/\n$/, '');
    //CERT = fs.readFileSync('./client.crt'), // todo: need to enable auth
    //KEY  = fs.readFileSync('./client.key'),
    //CA   = fs.readFileSync('./ca.crt');

var httpReq = function (method, path, postData, callback) {
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
  
  var req = http.request(options, function (res) {
    var resStr = "";

    res.on("data", function (chunk) {
        resStr += chunk;
    });

    res.on("end", function () {
      var obj = JSON.parse(resStr);
      //console.log('STATUS: ' + res.statusCode);
      //console.log('HEADERS: ' + JSON.stringify(res.headers));
      callback(obj);
    });
  });

  req.write(postData);

  req.end();

  req.on('error', function(e) {
    console.log('problem with request: ' + e.message);
  });
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
                    "water_potential_hydrogen" : "mega Pascals", // todo: confirm
                    "water_temperature" : "degrees celsius",
                    "water_level_high" : " ", // todo: confirm
                    "ph" : " "}
  return unitsMap[parameter];
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
                           'water potential hydrogen',
                           'water potential', // alias
                           'water temperature',
                           'water level high',
                           'water level hi', // alexa returns hi, not high
                           'ph level'];
  var isValidParameter = validParameters.indexOf(parameter) > -1; // true if a valid value was passed
  return isValidParameter;
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
                    "water potential" : "water_potential_hydrogen",
                    "water temperature" : "water_temperature",
                    "water level high" : "water_level_high",
                    "water level hi" : "water_level_high",
                    "ph level" : "pH_level"} // todo: check
  return paramMap[alexaParam];
}


function createNumberAttributes(key) {
    return {
        key : key
    };
}

// --------------- Helpers that build all of the responses ----------------------------

var buildSpeechletResponse = function (title, output, repromptText, shouldEndSession) {
    return {
        outputSpeech: {
            type: "PlainText",
            text: output
        },
        card: {
            type: "Simple",
            title: "SessionSpeechlet - " + title,
            content: "SessionSpeechlet - " + output
        },
        reprompt: {
            outputSpeech: {
                type: "PlainText",
                text: repromptText
            }
        },
        shouldEndSession: shouldEndSession
    };
}

function buildSpeechletResponseSSML(title, output, repromptText, shouldEndSession) {
    return {
        outputSpeech: {
            type: "SSML",
            "ssml": "<speak>"+output+"</speak>"
        },
        card: {
            type: "Simple",
            title: "SessionSpeechlet - " + title,
            content: "SessionSpeechlet - " + output
        },
        reprompt: {
            outputSpeech: {
                type: "SSML",
                "ssml": "<speak>"+repromptText+"</speak>"
            }
        },
        shouldEndSession: shouldEndSession
    };
}

function buildResponse(sessionAttributes, speechletResponse) {
    return {
        version: "1.0",
        sessionAttributes: sessionAttributes,
        response: speechletResponse
    };
}
