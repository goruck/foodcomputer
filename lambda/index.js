/**
 * Lambda function for OpenAg Personal Food Computer control and status triggered by Alexa.
 * 
 * For details see https://github.com/goruck.
 * 
 * Copyright (c) 2017 Lindo St. Angel
 */

//==============================================================================
//========================== Setup and Globals  ================================
//==============================================================================
'use strict';
var Alexa = require('alexa-sdk');
var APP_ID = "amzn1.ask.skill.cca03904-f750-41cc-a7cf-beb59fc21820";
var speechOutput = "";
var welcomeOutput = "Please ask the foodcomputer something.";
var welcomeReprompt = "You can say the name of a recipe to start,"
                      +" or ask for the value of a desired or measured parameter,"
                      +" or ask for diagnostics.";
/*If you don't want to use cards in your skill, set the USE_IMAGES_FLAG to false.
If you set it to true, you will need an image for each item in your data.*/
const USE_IMAGES_FLAG = true;

//==============================================================================
//========================== Event Handlers  ===================================
//==============================================================================
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
                  postData = '[\"' + recipe + '\"]',
                  text     = true;

            if (retObj.confirmationStatus === "CONFIRMED") {
                // Note: arrow functions don't actually bind a this value...
                httpsReq (method, path, postData, text, (resStr) => {
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
              path     = "/_openag/api/0.0.1/topic_data/arduino_status",
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (resStr) => {
            var obj = JSON.parse(resStr);

            if (obj.result === "OK") {
                speechOutput =  "The food computer is operating normally.";
            } else { // the arduino has a warning or an error
                speechOutput = obj.result;
            }

            this.emit(":tell", speechOutput);
        });
    },
    "ShowImage": function() {
        console.log("Show Image event: " + JSON.stringify(this.event));
        const s3ImagePath = "https://s3.amazonaws.com/" + s3BucketName + "/";
        const s3ImageName = "pfc-image.png";
        var cameraName = this.event.request.intent.slots.camera.value.toLowerCase();

        // Check validity of user request and map to food computer db var names.
        if (cameraName === "top" || cameraName === "side") {
            var _camera = (cameraName === "top" ? "aerial_image" : "side_image");
            // Check to see if device has a display (or is the simulator).
            if (supportsDisplay.call(this) || isSimulator.call(this)) {
                // Query food computer database for requested image and get path of the most recent.
                var queryPath = "/environmental_data_point/_design/openag/_view/by_variable?reduce=true" +
                                "\&startkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22]" +
                                "\&endkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22,{}]";
                httpsReq("GET", queryPath, "", true, (result) => {
                    var obj = safelyParseJSON(result);
                    if (obj && obj.rows[0]) {
                        var ts = obj.rows[0].value.timestamp; // timestamp of most recent image in Unix epoch time
                        //console.log("ts: " + ts);
                        var id = obj.rows[0].value._id; // id of most recent image
                        //console.log("id: " + id);
                        var pfcImagePath = "/environmental_data_point/" + id + "/image";

                        // Get image from food computer db, place in S3 bucket and display it on a capable device.
                        httpsReq("GET", pfcImagePath, "", false, (pfcImage) => {
                            if (isPng(pfcImage)) {
                                puts3File(s3ImageName, pfcImage, (err, data) => {
                                    if (err) {
                                        console.log("ERROR ShowImage: puts3File error: " + err);
                                        this.response.speak("sorry, I can't complete the request");
                                        this.emit(":responseReady");
                                    } else {
                                        let content = {
                                                       "hasDisplaySpeechOutput" : "showing" + cameraName + "camera",
                                                       "bodyTemplateContent" : cameraName,
                                                       "templateToken" : "ShowImage",
                                                       "askOrTell": ":tell",
                                                       "sessionAttributes" : this.attributes
                                        };
                                        if (USE_IMAGES_FLAG) {
                                            content["backgroundImageUrl"] = s3ImagePath + s3ImageName;
                                        }
                                        renderTemplate.call(this, content);
                                    }
                                });
                            } else {
                                console.log("ERROR ShowImage: https request didn't get a valid png image");
                                this.response.speak("sorry, I can't complete the request");
                                this.emit(":responseReady");
                            }
                        });
                    } else {
                        console.log("ERROR ShowImage: food computer db query didn't return anything");
                        this.response.speak("sorry, I can't complete the request");
                        this.emit(":responseReady");
                    }
                });
            } else { // Device does not have a display. 
                this.response.speak("sorry, cannot show image since this device has no display");
                this.emit(":responseReady");
            }
        } else { // Bad request.
            console.log("ERROR ShowImage: bad request: " + cameraName);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
        }
    },
    '_GetDiagnosticInfo': function () { // not using - looks like the ros topic /diagnostic is deprecated?
        const method   = "GET",
              path     = "/_openag/api/0.0.1/topic_data/diagnostics",
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (resStr) => {
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

//==============================================================================
//===================== Food Computer Helper Functions  ========================
//==============================================================================
/*
 *
 */
function getParameterValue(desired /*true if desired*/, parameter) { // openag brain api version
    // Check user request for validity.
    var parameterLowerCase = parameter.toLowerCase(); // no guarantee that Alexa ASR will return value in lower case
    var isValidParameter = checkIfParamIsValid(parameterLowerCase);

    if (isValidParameter) { // parameter valid
        var foodcomputerParam = alexaParamToFoodcomputerParam(parameterLowerCase);
        const method   = "GET",
              pathBase = "/_openag/api/0.0.1/topic_data/environments/environment_1/",
              path     = pathBase + foodcomputerParam + (desired ? "/desired" : "/measured"),
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (resStr) => {
            var obj = JSON.parse(resStr);
            var paramValue = NaN;
            
            paramValue = obj.result;

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
 *
 */
function _getParameterValue(desired /*true if desired*/, parameter) { // CouchDB version - can be very slow
    // Check user request for validity.
    var parameterLowerCase = parameter.toLowerCase(); // no guarantee that Alexa ASR will return value in lower case
    var isValidParameter = checkIfParamIsValid(parameterLowerCase);

    if (isValidParameter) { // parameter valid
        var foodcomputerParam = alexaParamToFoodcomputerParam(parameterLowerCase);
        const method   = "GET",
              path     = "/environmental_data_point/_design/openag/_view/by_variable?group_level=3",
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (resStr) => {
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
    HOST = fs.readFileSync('./host.txt').toString().replace(/\n$/, ''),
    CERT = fs.readFileSync('./certs/client.crt'),
    KEY  = fs.readFileSync('./certs/client.key'),
    CA   = fs.readFileSync('./certs/ca.crt');

var httpsReq = (method, path, postData, text, callback) => {
    var options = {
        hostname: HOST,
        port: PORT,
        path: path,
        method: method,
        rejectUnauthorized: true,
        key: KEY,
        cert: CERT,
        ca: CA,
        headers: {
            "Content-Type": (text ? "application/json" : "image/png"),
            "Content-Length": postData.length
        }
    };

    var https = require('https');
    var Stream = require("stream").Transform;

    var req = https.request(options, (result) => {
        var data = new Stream();

        result.on("data", (chunk) => {
            data.push(chunk);
        });

        result.on("end", () => {
            //console.log('STATUS: ' + result.statusCode);
            //console.log('HEADERS: ' + JSON.stringify(result.headers));
            callback(data.read());
        });
    });

    req.write(postData);

    req.end();

    req.on("error", (e) => {
        console.log("ERROR https request: " + e.message);
    });
}

//==============================================================================
//==================== Alexa Delegate Helper Functions  ========================
//==============================================================================
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

//==============================================================================
//============================ S3 Helper Functions  ============================
//==============================================================================
var AWS = require("aws-sdk");
var s3 = new AWS.S3();
const s3BucketName = "foodcomputer-images";

// Get file from S3
function getS3File(fileName, versionId, callback) {
    var params = {
        Bucket: s3BucketName,
        Key: fileName
    };
    if (versionId) {
        params.VersionId = versionId;
    }
    s3.getObject(params, function (err, data) {
        callback(err, data);
    });
}

// Put file into S3
function puts3File(fileName, data, callback) {
    var expirationDate = new Date();
    // Assuming a user would not remain active in the same session for over 1 hr.
    expirationDate = new Date(expirationDate.setHours(expirationDate.getHours() + 1));
    var params = {
        Bucket: s3BucketName,
        Key: fileName,
        Body: data,
        ACL: "public-read", // TODO: find way to restrict access to this lambda function
        Expires: expirationDate
    };
    s3.putObject(params, function (err, data) {
        callback(err, data);
    });
}

//==============================================================================
//===================== Echo Show Helper Functions  ============================
//==============================================================================
function supportsDisplay() {
  var hasDisplay =
    this.event.context &&
    this.event.context.System &&
    this.event.context.System.device &&
    this.event.context.System.device.supportedInterfaces &&
    this.event.context.System.device.supportedInterfaces.Display

  return hasDisplay;
}

function isSimulator() {
  var isSimulator = !this.event.context; //simulator doesn't send context
  return false;
}


function renderTemplate (content) {
   console.log("renderTemplate" + content.templateToken);
   //learn about the various templates
   //https://developer.amazon.com/public/solutions/alexa/alexa-skills-kit/docs/display-interface-reference#display-template-reference
   //
   switch(content.templateToken) {
       case "WelcomeScreenView":
         //Send the response to Alexa
         this.context.succeed(response);
         break;
       case "ShowImage":
        //  "hasDisplaySpeechOutput" : response + " " + EXIT_SKILL_MESSAGE,
        //  "bodyTemplateContent" : getFinalScore(this.attributes["quizscore"], this.attributes["counter"]),
        //  "templateToken" : "FinalScoreView",
        //  "askOrTell": ":tell",
        //  "hint":"start a quiz",
        //  "sessionAttributes" : this.attributes
        //  "backgroundImageUrl"
        var response = {
          "version": "1.0",
          "response": {
            "directives": [
              {
                "type": "Display.RenderTemplate",
                "backButton": "HIDDEN",
                "template": {
                  "type": "BodyTemplate6",
                  //"title": content.bodyTemplateTitle,
                  "token": content.templateToken,
                  "textContent": {
                    "primaryText": {
                      "type": "RichText",
                      "text": "<font size = '7'>"+content.bodyTemplateContent+"</font>"
                    }
                  }
                }
              },{
                  "type": "Hint",
                  "hint": {
                    "type": "PlainText",
                    "text": content.hint
                  }
                }
            ],
            "outputSpeech": {
              "type": "SSML",
              "ssml": "<speak>"+content.hasDisplaySpeechOutput+"</speak>"
            },
            "reprompt": {
              "outputSpeech": {
                "type": "SSML",
                "ssml": ""
              }
            },
            "shouldEndSession": content.askOrTell== ":tell",

          },
          "sessionAttributes": content.sessionAttributes

        }

        if(content.backgroundImageUrl) {
          //when we have images, create a sources object

          let sources = [
            {
              "size": "SMALL",
              "url": content.backgroundImageUrl
            },
            {
              "size": "LARGE",
              "url": content.backgroundImageUrl
            }
          ];
          //add the image sources object to the response
          response["response"]["directives"][0]["template"]["backgroundImage"]={};
          response["response"]["directives"][0]["template"]["backgroundImage"]["sources"]=sources;
        }



         //Send the response to Alexa
         this.context.succeed(response);
         break;

       case "ItemDetailsView":
           var response = {
             "version": "1.0",
             "response": {
               "directives": [
                 {
                   "type": "Display.RenderTemplate",
                   "template": {
                     "type": "BodyTemplate3",
                     "title": content.bodyTemplateTitle,
                     "token": content.templateToken,
                     "textContent": {
                       "primaryText": {
                         "type": "RichText",
                         "text": "<font size = '5'>"+content.bodyTemplateContent+"</font>"
                       }
                     },
                     "backButton": "HIDDEN"
                   }
                 }
               ],
               "outputSpeech": {
                 "type": "SSML",
                 "ssml": "<speak>"+content.hasDisplaySpeechOutput+"</speak>"
               },
               "reprompt": {
                 "outputSpeech": {
                   "type": "SSML",
                   "ssml": "<speak>"+content.hasDisplayRepromptText+"</speak>"
                 }
               },
               "shouldEndSession": content.askOrTell== ":tell",
               "card": {
                 "type": "Simple",
                 "title": content.simpleCardTitle,
                 "content": content.simpleCardContent
               }
             },
             "sessionAttributes": content.sessionAttributes

         }

          if(content.imageSmallUrl && content.imageLargeUrl) {
            //when we have images, create a sources object
            //TODO switch template to one without picture?
            let sources = [
              {
                "size": "SMALL",
                "url": content.imageSmallUrl
              },
              {
                "size": "LARGE",
                "url": content.imageLargeUrl
              }
            ];
            //add the image sources object to the response
            response["response"]["directives"][0]["template"]["image"]={};
            response["response"]["directives"][0]["template"]["image"]["sources"]=sources;
          }
          //Send the response to Alexa
          console.log("ready to respond (ItemDetailsView): "+JSON.stringify(response));
           this.context.succeed(response);
           break;
       case "MultipleChoiceListView":
       console.log ("listItems "+JSON.stringify(content.listItems));
           var response = {
              "version": "1.0",
              "response": {
                "directives": [
                  {
                    "type": "Display.RenderTemplate",
                    "template": {
                      "type": "ListTemplate1",
                      "title": content.listTemplateTitle,
                      "token": content.templateToken,
                      "listItems":content.listItems,
                      "backgroundImage": {
                        "sources": [
                          {
                            "size": "SMALL",
                            "url": content.backgroundImageSmallUrl
                          },
                          {
                            "size": "LARGE",
                            "url": content.backgroundImageLargeUrl
                          }
                        ]
                      },
                      "backButton": "HIDDEN"
                    }
                  }
                ],
                "outputSpeech": {
                  "type": "SSML",
                  "ssml": "<speak>"+content.hasDisplaySpeechOutput+"</speak>"
                },
                "reprompt": {
                  "outputSpeech": {
                    "type": "SSML",
                    "ssml": "<speak>"+content.hasDisplayRepromptText+"</speak>"
                  }
                },
                "shouldEndSession": content.askOrTell== ":tell",
                "card": {
                  "type": "Simple",
                  "title": content.simpleCardTitle,
                  "content": content.simpleCardContent
                }
              },
                "sessionAttributes": content.sessionAttributes

          }

            if(content.backgroundImageLargeUrl) {
              //when we have images, create a sources object
              //TODO switch template to one without picture?
              let sources = [
                {
                  "size": "SMALL",
                  "url": content.backgroundImageLargeUrl
                },
                {
                  "size": "LARGE",
                  "url": content.backgroundImageLargeUrl
                }
              ];
              //add the image sources object to the response
              response["response"]["directives"][0]["template"]["backgroundImage"]={};
              response["response"]["directives"][0]["template"]["backgroundImage"]["sources"]=sources;
            }
            console.log("ready to respond (MultipleChoiceList): "+JSON.stringify(response));
           this.context.succeed(response);
           break;
       default:
          this.response.speak("Thanks for playing, goodbye");
          this.emit(':responseReady');
   }

}

//==============================================================================
//======================== Misc Helper Functions  ==============================
//==============================================================================
/*
 * Checks if a file is a png image.
 * https://stackoverflow.com/questions/8473703/in-node-js-given-a-url-how-do-i-check-whether-its-a-jpg-png-gif/8475542#8475542
 */
function isPng(file) {
    const pngMagicNum = "89504e47";
    var magicNumInFile = file.toString('hex',0,4);
    //console.log("magicNumInFile: " + magicNumInFile);
  
    if (magicNumInFile === pngMagicNum) {
        return true;
    } else {
        return false;
    }
}
/*
 * Checks for valid JSON. 
 */
function safelyParseJSON(json) {
    var parsed;

    try {
        return parsed = JSON.parse(json)
    } catch (e) {
        return null;
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
