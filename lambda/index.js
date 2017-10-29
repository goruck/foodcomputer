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
var fs = require('fs');
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
                httpsReq (method, path, postData, text, (err, resStr) => {
                    if (err) {
                        console.log("ERROR StartRecipe: httpsReq: " + err);
                        this.response.speak("sorry, I can't complete the request");
                        this.emit(":responseReady");
                        return;
                    }

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
    'GetParameterValue': function () {
        console.log("GetParameterValue event: " + JSON.stringify(this.event));
        var parameter = this.event.request.intent.slots.parameter.value;
        getParameterValue.call(this, parameter);
    },
    'GetDiagnosticInfo': function () {
        const method   = "GET",
              path     = "/_openag/api/0.0.1/topic_data/arduino_status",
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (err, resStr) => {
            if (err) {
                console.log("ERROR GetDiagnostics: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            var obj = JSON.parse(resStr);
            if (obj.data === "OK") {
                speechOutput =  "The food computer is operating normally.";
            } else { // the arduino has a warning or an error
                speechOutput = obj.data;
            }

            var d = new Date();
            var n = d.getTime();
            var dateTime = timeConverter(n / 1000);

            if (supportsDisplay.call(this) || isSimulator.call(this)) {
                let content = {
                    "hasDisplaySpeechOutput" : speechOutput,
                    "title" : "Diagnostic information at "+dateTime+".",
                    "textContent" : speechOutput,
                    "templateToken" : "SingleItemView",
                    "askOrTell": ":tell",
                    "sessionAttributes" : this.attributes
                };
                renderTemplate.call(this, content);
            } else {
                this.emit(":tell", speechOutput);
            }
        });
    },
    "ShowImage": function() { // couchDB version
        console.log("Show Image event: " + JSON.stringify(this.event));

        // Check to see if device has a display (or is the simulator).
        if (!supportsDisplay.call(this) && !isSimulator.call(this)) {
            this.response.speak("sorry, cannot show image since this device has no display");
            this.emit(":responseReady");
            return;
        }

        // Default to top camera normal view if camera slot comes in undefined
        var cameraView = "";
        if (this.event.request.intent.slots.camera.value === undefined) {
            cameraView = "top"; 
        } else {
            cameraView = this.event.request.intent.slots.camera.value.toLowerCase();
        }

        // Check validity of user request and map to food computer db var names.
        const cameraMapping = {
            "top" : "aerial_image/image_raw",
            "side" : "frontal_image/image_raw",
            "measurement" : "aerial_image/image_rect_color/ObjectDetectorAndBlobDetector"
        }
        if (!cameraMapping.hasOwnProperty(cameraView)) {
            // Bad request.
            console.log("ERROR ShowImage: bad request: " + cameraView);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        var _camera = cameraMapping[cameraView];
        var queryPath = "";
        if (this.event.request.intent.slots.duration.value === undefined) { // Defaults to now (most recent image)
            /*
             * Form CounchDB query for most recent image. 
             * Using "stale = update_after", CouchDB will update the view after the stale result is returned.
             * This will speed up queries but results in slighty out of data info returned.
             * See 'https://wiki.apache.org/couchdb/HTTP_view_API#Querying_Options'.
             */
            queryPath = "/environmental_data_point/_design/openag/_view/by_variable?" +
                        "reduce=true\&stale=update_after" +
                        "\&startkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22]" +
                        "\&endkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22,{}]";
        } else {
            var durationString = this.event.request.intent.slots.duration.value; // A duration string in ISO 8601 format

            var duration = parseISO8601Duration(durationString);

            if (duration.years || duration.months) { // Too far back...
                console.log("ERROR ShowImage: Requested view too long ago");
                inspectLogObj(duration, 1);
                this.response.speak("Sorry, I can't complete the request. That's too long ago.");
                this.emit(":responseReady");
                return;
            }

            // Get current Epoch timestamp in seconds.
            var d = new Date();
            var currentTs = d.getTime() / 1000;

            // Calculate query start and end times based on current timestamp.
            var queryStartTime = currentTs - (duration.weeks * 604800)
                                           - (duration.days  * 86400)
                                           - (duration.hours * 3600)
                                           - (duration.minutes * 60)
                                           - (duration.seconds);
            // Make query span one hour - should catch at least one image.
            var queryEndTime = queryStartTime + 3600;
            //console.log("qstart: "+queryStartTime+" qend: "+queryEndTime);

            // Form CouchDB query for past image. 
            queryPath = "/environmental_data_point/_design/openag/_view/by_variable?" +
                        "reduce=false" +
                        "\&startkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22,"+queryStartTime+"]" +
                        "\&endkey=[%22environment_1%22,%22measured%22,%22"+_camera+"%22,"+queryEndTime+"]";
        }
        
        // Query food computer database for requested image and get its ID and timestamp.
        httpsReq("GET", queryPath, "", true, (err, result) => {
            if (err) {
                console.log("ERROR ShowImage: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            var obj = safelyParseJSON(result);
            if (!obj || !obj.rows[0]) {
                console.log("ERROR ShowImage: food computer db query didn't return anything");
                this.response.speak("Sorry, I can't complete the request. No image was found.");
                this.emit(":responseReady");
                return;
            }

            // Actual timestamp of image in Unix epoch time. Should be close to what user requested. 
            var imageTs = obj.rows[0].value.timestamp;
            // ID of image.
            var imageId = obj.rows[0].value._id;

            // Path to requested image. 
            var pfcImagePath = "/environmental_data_point/" + imageId + "/image";

            // Get image from food computer db, place in S3 bucket and display it on a capable device.
            httpsReq("GET", pfcImagePath, "", false, (err, pfcImage) => {
                if (err) {
                    console.log("ERROR ShowImage: httpsReq: " + err);
                    this.response.speak("sorry, I can't complete the request");
                    this.emit(":responseReady");
                    return;
                 }

                 if (!isPng(pfcImage)) {
                     console.log("ERROR ShowImage: https request didn't get a valid png image");
                     this.response.speak("sorry, I can't complete the request");
                     this.emit(":responseReady");
                     return;
                 }

                 const s3BucketName = "foodcomputer-images";
                 const s3ImagePath = "https://s3.amazonaws.com/" + s3BucketName + "/";
                 const s3ImageName = "pfc-image.png";
                 putS3File(s3BucketName, s3ImageName, pfcImage, (err, data) => {
                     if (err) {
                         console.log("ERROR ShowImage: puts3File error: " + err);
                         this.response.speak("sorry, I can't complete the request");
                         this.emit(":responseReady");
                         return;
                     }

                     let content = {
                         "hasDisplaySpeechOutput" : "showing" + cameraView + "view",
                         "bodyTemplateContent" : timeConverter(imageTs),
                         "templateToken" : "ShowImage",
                         "askOrTell": ":tell",
                         "sessionAttributes" : this.attributes
                     };
                     if (USE_IMAGES_FLAG) {
                        content["backgroundImageUrl"] = s3ImagePath + s3ImageName;
                     }
                     renderTemplate.call(this, content);
                });
            });
        });
    },
    "_ShowImage": function() { // openag api version
        console.log("Show Image event: " + JSON.stringify(this.event));

        // Check to see if device has a display (or is the simulator).
        if (!supportsDisplay.call(this) && !isSimulator.call(this)) {
            this.response.speak("sorry, cannot show image since this device has no display");
            this.emit(":responseReady");
            return;
        }
        
        // Default to top camera normal view if slot comes in undefined
        var cameraView = "";
        if (this.event.request.intent.slots.camera.value === undefined) {
            cameraView = "top"; 
        } else {
            cameraView = this.event.request.intent.slots.camera.value.toLowerCase();
        }

        // Check validity of user request and map to food computer db var names.
        const cameraMapping = {
            "top" : "aerial_image/image_raw",
            "side" : "frontal_image/image_raw",
            "measurement" : "aerial_image/image_rect_color/ObjectDetectorAndBlobDetector"
        }
        if (!cameraMapping.hasOwnProperty(cameraView)) {
            // Bad request.
            console.log("ERROR ShowImage: bad request: " + cameraView);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }
        const path = "/_openag/api/0.0.1/topic_data";
        var pfcImagePath = path + "/environments/environment_1/" + cameraMapping[cameraView];
        
        // Get image from food computer openag api, place in S3 bucket and display it on a capable device.
        httpsReq("GET", pfcImagePath, "", false, (err, pfcImage) => {
            if (err) {
                console.log("ERROR ShowImage: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
             }

             if (!isPng(pfcImage)) {
                 console.log("ERROR ShowImage: https request didn't get a valid png image");
                 this.response.speak("sorry, I can't complete the request");
                 this.emit(":responseReady");
                 return;
             }

             const s3BucketName = "foodcomputer-images";
             const s3ImagePath = "https://s3.amazonaws.com/" + s3BucketName + "/";
             const s3ImageName = "pfc-image.png";
             putS3File(s3BucketName, s3ImageName, pfcImage, (err, data) => {
                 if (err) {
                     console.log("ERROR ShowImage: puts3File error: " + err);
                     this.response.speak("sorry, I can't complete the request");
                     this.emit(":responseReady");
                     return;
                 }

                 var d = new Date();
                 var n = d.getTime();
                 var dateTime = timeConverter(n / 1000);

                 let content = {
                     "hasDisplaySpeechOutput" : "showing" + cameraView + "view",
                     "bodyTemplateContent" : dateTime,
                     "templateToken" : "ShowImage",
                     "askOrTell": ":tell",
                     "sessionAttributes" : this.attributes
                 };
                 if (USE_IMAGES_FLAG) {
                    content["backgroundImageUrl"] = s3ImagePath + s3ImageName;
                 }
                 renderTemplate.call(this, content);
            });
        });
    },
    "ShowGraph": function() {
        console.log("Show Graph event: " + JSON.stringify(this.event));

        // Check to see if device has a display (or is the simulator).
        if (!supportsDisplay.call(this) && !isSimulator.call(this)) {
            this.response.speak("sorry, cannot show image since this device has no display");
            this.emit(":responseReady");
            return;
        }

        // Check for valid slot. 
        if (this.event.request.intent.slots.parameter.value !== undefined) {
           var parameter = this.event.request.intent.slots.parameter.value.toLowerCase();
           //console.log("parameter: " +parameter);
        } else {
            console.log("ERROR ShowGraph: slot undefined");
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }
        
        // Check user request for validity and map to PFC parameter name.
        if (checkIfParamIsValid(parameter)) {
            var pfcParam = alexaParamToFoodcomputerParam(parameter);
            //console.log("pfcParam: " +pfcParam);
        } else {
            console.log("ERROR ShowGraph: bad request: " + parameter);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        // Calculate start and stop graph times from current time. 
        var d = new Date();
        var n = d.getTime(); // Number of milliseconds since 1970/01/01.
        const daysToGraph = 1; // Number of days to graph the parameter over. 
        var deltaMs = daysToGraph * 24 * 60 * 60 * 1000; // Days in ms. 
        var pfcStartTime = (n - deltaMs) / 1000; // TODO make a slot
        //console.log("pfcStartTime: " +pfcStartTime);
        var pfcEndTime = n / 1000; // TODO make a slot
        //console.log("pfcEndTime: " +pfcEndTime);

        /*
         * Query food computer database for requested variable data over the time range.
         * Using "stale = update_after", CouchDB will update the view after the stale result is returned.
         * This will speed up queries but results in slighty out of data info returned.
         * See 'https://wiki.apache.org/couchdb/HTTP_view_API#Querying_Options'.
         */
        // JSON query - about 4.5 MB for one day's worth of data.
        var queryPathJson = "/environmental_data_point/_design/openag/_view/by_variable?" +
                            "reduce=false\&stale=update_after" +
                            "\&startkey=[%22environment_1%22,%22measured%22,%22"+pfcParam+"%22,"+pfcStartTime+"]" +
                            "\&endkey=[%22environment_1%22,%22measured%22,%22"+pfcParam+"%22,"+pfcEndTime+"]";

        // CSV query - about 0.45 MB for one day's worth of data, using this one. 
        var queryPathCsv = "/environmental_data_point/_design/openag/_list/csv/by_variable?" +
                           "reduce=false\&stale=update_after" +
                           "\&startkey=[%22environment_1%22,%22measured%22,%22"+pfcParam+"%22,"+pfcStartTime+"]" +
                           "\&endkey=[%22environment_1%22,%22measured%22,%22"+pfcParam+"%22,"+pfcEndTime+"]" +
                           "\&cols=[%22timestamp%22,%22value%22]";

        console.time("httpsReq");
        httpsReq("GET", queryPathCsv, "", true, (err, result) => {
            console.timeEnd("httpsReq");
            if (err) {
                console.log("ERROR ShowGraph: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            var figure = mkPlotlyFigFromCsv(result, parameter, "time", getParamUnits(pfcParam));
            //var figure = mkPlotlyFigFromJson(result, parameter, "time", getParamUnits(pfcParam);
            const imgOpts = {
                  format: "png",
                  width: 1024,
                  height: 600
            };
            var PLOTLY_USER = fs.readFileSync("./plotly/username.txt").toString().replace(/\n$/, '');
            var PLOTLY_KEY  = fs.readFileSync("./plotly/apikey.txt").toString().replace(/\n$/, '');
            var plotly = require("plotly")(PLOTLY_USER, PLOTLY_KEY);
            console.time("plotly.getImage");
            plotly.getImage(figure, imgOpts, (err, imageStream) => {
                console.timeEnd("plotly.getImage");
                if (err) {
                    console.log("ERROR ShowGraph: plotly: " + err);
                    this.response.speak("sorry, I can't complete the request");
                    this.emit(":responseReady");
                    return;
                }

                const s3BucketName = "foodcomputer-graphs";
                const s3ImagePath = "https://s3.amazonaws.com/" + s3BucketName + "/";
                const s3ImageName = "pfc-graph.png";
                uploadS3File(s3BucketName, s3ImageName, imageStream, (err, data) => {
                    if (err) {
                        console.log("ERROR ShowImage: uploadS3File: " + err);
                        this.response.speak("sorry, I can't complete the request");
                        this.emit(":responseReady");
                        return;
                    }

                    let content = {
                        "hasDisplaySpeechOutput" : "showing graph of " + parameter,
                        "bodyTemplateContent" : "",
                        "templateToken" : "ShowImage",
                        "askOrTell": ":tell",
                        "sessionAttributes" : this.attributes
                    };
                    if (USE_IMAGES_FLAG) {
                        content["backgroundImageUrl"] = s3ImagePath + s3ImageName;
                    }
                    renderTemplate.call(this, content);
                });
            });
        });
    },
    "GetRecipeInfo": function() {
        console.log("Get recipe info event: " + JSON.stringify(this.event));

        /*
         * Query to food computer database for recipe information.
         * Using "stale = update_after", CouchDB will update the view after the stale result is returned.
         * This will speed up queries but results in slighty out of data info returned.
         * See 'https://wiki.apache.org/couchdb/HTTP_view_API#Querying_Options'.
         */
        var queryPath = "/environmental_data_point/_design/openag/_view/by_variable?stale=update_after\&group_level=3";

        httpsReq("GET", queryPath, "", true, (err, result) => {
            if (err) {
                console.log("ERROR GetRecipeInfo: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            var obj = safelyParseJSON(result);
            if (!obj || !obj.rows[0]) {
                console.log("ERROR GetRecipeInfo: food computer db query didn't return anything");
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            // Search json obj for recipe information.
            var recipeStartTimeStamp = 0;
            var recipeStartName      = "";
            var recipeEndTimeStamp   = 0;
            var recipeEndName        = "";
            for (var i = 0, len = obj.rows.length; i < len; i++) {
                if (obj.rows[i].key[1] === "desired" && obj.rows[i].key[2] === "recipe_start") {
                    recipeStartTimeStamp = obj.rows[i].value.timestamp;
                    recipeStartName = obj.rows[i].value.value;
                } else if (obj.rows[i].key[1] === "desired" && obj.rows[i].key[2] === "recipe_end") {
                    recipeEndTimeStamp = obj.rows[i].value.timestamp;
                    recipeEndName = obj.rows[i].value.value;
                }
            }

            var recipeStartDateTime = "";
            if (recipeStartTimeStamp > recipeEndTimeStamp) {
                recipeStartDateTime = timeConverter(recipeStartTimeStamp);
                speechOutput = "Recipe "+recipeStartName+" has been running since "+recipeStartDateTime+".";
            } else {
                speechOutput = "No recipe is currently running.";
            }

            var d = new Date();
            var n = d.getTime();
            var dateTime = timeConverter(n / 1000);

            if (supportsDisplay.call(this) || isSimulator.call(this)) {
                let content = {
                    "hasDisplaySpeechOutput" : speechOutput,
                    "title" : "Recipe information at "+dateTime+".",
                    "textContent" : speechOutput,
                    "templateToken" : "SingleItemView",
                    "askOrTell": ":tell",
                    "sessionAttributes" : this.attributes
                };
                renderTemplate.call(this, content);
            } else {
                this.emit(":tell", speechOutput);
            }
        });
    },
    '_GetDiagnosticInfo': function () { // not using - looks like the ros topic /diagnostic is deprecated?
        const method   = "GET",
              path     = "/_openag/api/0.0.1/topic_data/diagnostics",
              postData = "",
              text     = true;

        httpsReq (method, path, postData, text, (err, resStr) => {
            if (err) {
                console.log("ERROR ShowGraph: httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

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
    }
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
 * Get a value of a measured parameter and the desired value if set.
 * This uses OpenAg Brain APIs (which are proxied by CouchDB). 
 *
 */
function getParameterValue(parameter) {

    // Check user request for validity.
    if (!checkIfParamIsValid(parameter)) {
        console.log("ERROR getParameterValue: bad request: " + parameter);
        this.response.speak("error, " + parameter + " is not a valid parameter");
        this.emit(":responseReady");
        return;
    }

    // Convert user friendly names to food computer param names.
    var parameterLowerCase = parameter.toLowerCase();
    var foodcomputerParam = alexaParamToFoodcomputerParam(parameterLowerCase);

    // Get the value of a measured param.
    const method   = "GET",
          pathBase = "/_openag/api/0.0.1/topic_data/environments/environment_1/",
          postData = "",
          text     = true;
    var path = pathBase + foodcomputerParam + "/measured";
    httpsReq (method, path, postData, text, (err, resStr) => {
        if (err) {
            console.log("ERROR getParameterValue: measured httpsReq: " + err);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        var measuredObj = safelyParseJSON(resStr);
        if (measuredObj === null || measuredObj.hasOwnProperty("error")) {
            console.log("ERROR getParameterValue: measured parse json");
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        var measuredParamValue = measuredObj.data;

        // Get the value of the corresponding desired param (if set) and output resp.
        path = pathBase + foodcomputerParam + "/desired";
        httpsReq (method, path, postData, text, (err, resStr) => {
            if (err) {
                console.log("ERROR getParameterValue: desired httpsReq: " + err);
                this.response.speak("sorry, I can't complete the request");
                this.emit(":responseReady");
                return;
            }

            var desiredObj = safelyParseJSON(resStr);
            var desiredParamValue = NaN;
            if (desiredObj !== null && measuredObj.hasOwnProperty("data")) {
                desiredParamValue = desiredObj.data;
            }

            speechOutput = ""; // global vars get cached by lambda between runs...

            // Format measured parameter value response.
            var measuredParamValueOut = (measuredParamValue === 0 ? measuredParamValue : measuredParamValue.toFixed(2));
            speechOutput += "Measured "+parameterLowerCase+" is "+measuredParamValueOut+" "+getParamUnits(foodcomputerParam)+".";

            // If a desired parameter value was found, include in the response. 
            if (!isNaN(desiredParamValue)) {
                var desiredParamValueOut = (desiredParamValue === 0 ? desiredParamValue : desiredParamValue.toFixed(2));
                speechOutput += " Desired "+parameterLowerCase+" is "+desiredParamValueOut+" "+getParamUnits(foodcomputerParam)+".";
            }

            // Get timestamp of parameter value which is about the current time. 
            var d = new Date();
            var n = d.getTime(); // Number of milliseconds since 1970/01/01.
            var dateTime = timeConverter(n / 1000);

            // Output to an Alexa device.
            if (supportsDisplay.call(this) || isSimulator.call(this)) {
                let content = {
                    "hasDisplaySpeechOutput" : speechOutput,
                    "title" : "Parameter Value at " + dateTime + ".",
                    "textContent" : speechOutput,
                    "templateToken" : "SingleItemView",
                    "askOrTell": ":tell",
                    "sessionAttributes" : this.attributes
                };
                renderTemplate.call(this, content);
            } else {
                this.emit(":tell", speechOutput);
            }
        });
    });

}

/*
 * Get a value of a measured parameter and its desired value if set and relevant.
 * This uses CouchDB APIs.
 */
function _getParameterValue(parameter) {

    // Check user request for validity.
    if (!checkIfParamIsValid(parameter)) {
        console.log("ERROR getParameterValue: bad request: " + parameter);
        this.response.speak("error, " + parameter + " is not a valid parameter");
        this.emit(":responseReady");
        return;
    }

    // Query food computer CouchDB and search it for requested parameter.
    // Using "stale=update_after" which speeds things up but values returned may be out of date.
    const method   = "GET",
          path     = "/environmental_data_point/_design/openag/_view/by_variable?stale=update_after\&group_level=3",
          postData = "",
          text     = true;
    httpsReq (method, path, postData, text, (err, resStr) => {
        if (err) {
            console.log("ERROR getParameterValue: httpsReq: " + err);
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        var obj = safelyParseJSON(resStr);
        if (!obj) {
            console.log("ERROR getParameterValue: safelyParseJSON");
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        var parameterLowerCase = parameter.toLowerCase();
        var foodcomputerParam = alexaParamToFoodcomputerParam(parameter);

        // Search json obj for requested param values and recipe start / end info. 
        var desiredParamValue      = NaN;
        var desiredParamTimeStamp  = 0;
        var measuredParamValue     = NaN;
        var measuredParamTimeStamp = 0;
        var recipeStartTimeStamp   = 0;
        var recipeEndTimeStamp     = 0;
        for (var i = 0, len = obj.rows.length; i < len; i++) {
            if (obj.rows[i].value.variable===foodcomputerParam&&obj.rows[i].value.is_desired===true) {
                desiredParamValue = obj.rows[i].value.value;
                desiredParamTimeStamp = timeConverter(obj.rows[i].value.timestamp);
                continue;
            } else if (obj.rows[i].value.variable===foodcomputerParam&&obj.rows[i].value.is_desired===false) {
                measuredParamValue = obj.rows[i].value.value;
                measuredParamTimeStamp = timeConverter(obj.rows[i].value.timestamp);
                continue;
            } else if (obj.rows[i].key[1]==="desired"&&obj.rows[i].key[2]==="recipe_start") {
                recipeStartTimeStamp = obj.rows[i].value.timestamp;
                continue;
            } else if (obj.rows[i].key[1]==="desired"&&obj.rows[i].key[2]==="recipe_end") {
                recipeEndTimeStamp = obj.rows[i].value.timestamp;
                continue;
            }
        }

        // There must be at least one measured value otherwise some error happened.
        if (isNaN(measuredParamValue)) {
            console.log("ERROR getParameterValue: no measured parm");
            this.response.speak("sorry, I can't complete the request");
            this.emit(":responseReady");
            return;
        }

        speechOutput = ""; // global vars get cached by lambda between runs...

        // Format measured parameter value response.
        var measuredParamValueOut = (measuredParamValue === 0 ? measuredParamValue : measuredParamValue.toFixed(2));
        speechOutput += "Measured "+parameterLowerCase+" is "+measuredParamValueOut+" "+getParamUnits(foodcomputerParam)+".";

        // If a desired parameter value was found and a recipe is running include in the response.
        // (If a recipe isn't running ignore desired values because they are no longer in effect.)
        if (!isNaN(desiredParamValue) && (recipeStartTimeStamp > recipeEndTimeStamp)) {
            var desiredParamValueOut = (desiredParamValue === 0 ? desiredParamValue : desiredParamValue.toFixed(2));
            speechOutput += " Desired "+parameterLowerCase+" is "+desiredParamValueOut+" "+getParamUnits(foodcomputerParam)+".";
        }

        // Output to an Alexa device. 
        if (supportsDisplay.call(this) || isSimulator.call(this)) {
            let content = {
                "hasDisplaySpeechOutput" : speechOutput,
                "title" : "Measured parameter value at "+measuredParamTimeStamp+".",
                "textContent" : speechOutput,
                "templateToken" : "SingleItemView",
                "askOrTell": ":tell",
                "sessionAttributes" : this.attributes
            };
            renderTemplate.call(this, content);
        } else {
            this.emit(":tell", speechOutput);
        }
    });

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
                    "air_temperature" : "degrees Celsius",
                    "light_illuminance" : "lux", // todo: confirm
                    "light_intensity_blue" : "lux",
                    "light_intensity_red" : "lux",
                    "light_intensity_white" : "lux",
                    "water_electrical_conductivity" : "micro Siemens per centimeter",
                    "water_potential_hydrogen" : " ", // aka pH
                    "water_temperature" : "degrees Celsius",
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
                    "water level hi" : "water_level_high",
                    "recipe" : "recipe_start"};
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
var httpsReq = (method, path, postData, text, callback) => {
    var PORT = fs.readFileSync('./port.txt').toString().replace(/\n$/, ''), // Ignore last newline character
        HOST = fs.readFileSync('./host.txt').toString().replace(/\n$/, ''),
        CERT = fs.readFileSync('./certs/client.crt'),
        KEY  = fs.readFileSync('./certs/client.key'),
        CA   = fs.readFileSync('./certs/ca.crt');

    var https = require('https'),
        Stream = require("stream").Transform,
        zlib = require('zlib');

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
            "Content-Length": postData.length,
            "accept-encoding" : "gzip,deflate"
        }
    };

    var req = https.request(options, (result) => {
        var data = new Stream();

        result.on("data", (chunk) => {
            data.push(chunk);
            //console.log("chunk: " +chunk);
        });

        result.on("end", () => {
            //console.log("STATUS: " + result.statusCode);
            //console.log("HEADERS: " + JSON.stringify(result.headers));

            var encoding = result.headers["content-encoding"];
            if (encoding == "gzip") {
                zlib.gunzip(data.read(), function(err, decoded) {
                    callback(null, decoded); // TODO: add error handleing.
                });
            } else if (encoding == "deflate") {
                zlib.inflate(data.read(), function(err, decoded) {
                    callback(null, decoded);
                });
            } else {
                callback(null, data.read());
            }

           //callback(data.read());
        });
    });

    // Set timeout on socket inactivity. 
    req.on("socket", function (socket) {
        socket.setTimeout(10000); // 10 sec timeout. 
        socket.on("timeout", function() {
            req.abort();
        });
    });

    req.write(postData);

    req.end();

    req.on("error", (e) => {
        console.log("ERROR https request: " + e.message);
        callback(e.message, null);
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

// Get file from S3
function getS3File(bucketName, fileName, versionId, callback) {
    var params = {
        Bucket: bucketName,
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
function putS3File(bucketName, fileName, data, callback) {
    var expirationDate = new Date();
    // Assuming a user would not remain active in the same session for over 1 hr.
    expirationDate = new Date(expirationDate.setHours(expirationDate.getHours() + 1));
    var params = {
        Bucket: bucketName,
        Key: fileName,
        Body: data,
        ACL: "public-read", // TODO: find way to restrict access to this lambda function
        Expires: expirationDate
    };
    s3.putObject(params, function (err, data) {
        callback(err, data);
    });
}

// Upload object to S3
function uploadS3File(bucketName, fileName, data, callback) {
    var params = {
        Bucket: bucketName,
        Key: fileName,
        Body: data,
        ACL: "public-read", // TODO: find way to restrict access to this lambda function
    };
    s3.upload(params, function(err, data) {
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
                  //"title": content.title,
                  "token": content.templateToken,
                  "textContent": {
                    "primaryText": {
                      "type": "RichText",
                      "text": "<font size = '3'>"+content.bodyTemplateContent+"</font>"
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
       case "SingleItemView":
           var response = {
              "version": "1.0",
              "response": {
                "directives": [
                  {
                    "type": "Display.RenderTemplate",
                    "template": {
                      "type": "BodyTemplate1",
                      "title": content.title,
                      "token": content.templateToken,
                      "textContent": {
                        "primaryText": {
                        "type": "RichText",
                        "text": "<font size = '7'>"+content.textContent+"</font>"
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
                    "ssml": ""
                  }
                },
                "shouldEndSession": content.askOrTell== ":tell",
              },
              "sessionAttributes": content.sessionAttributes
          }
           console.log("ready to respond (SingleItemView): "+JSON.stringify(response));
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
 * Converts Unix timestamp in seconds to normal date and time of day.
 */
function timeConverter(unix_timestamp) {
    const months = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];
    const tzDiff = 7 * 60 * 60; // Pacific time is 7 hours behind UTC
    // Create a new JavaScript Date object based on the timestamp
    // multiplied by 1000 so that the argument is in milliseconds, not seconds.
    var date = new Date( ( unix_timestamp - tzDiff ) * 1000 );
    var year = date.getFullYear();
    //var month = months[date.getMonth()];
    var month = 1 + date.getMonth();
    var day = date.getDate();
    var hours = date.getHours();
    var minutes = "0" + date.getMinutes();
    var seconds = "0" + date.getSeconds();

   // Will display time in M D HH:MM format
   //var formattedTime = month + " " + day + " " + hours + ":" + minutes.substr(-2);
   // Will display in 2013-10-04 22:23:00 format
   var formattedTime = year+"-"+month+"-"+day+" "+hours+":"+minutes.substr(-2)+":"+seconds.substr(-2);
   return formattedTime;
}

/*
 * Parse ISO8501 duration string.
 * See https://stackoverflow.com/questions/27851832/how-do-i-parse-an-iso-8601-formatted-duration-using-moment-js
 *
 */
function parseISO8601Duration(durationString) {
    // regex to parse ISO8501 duration string.
    // TODO: optimize regex since it matches way more than needed.
    var iso8601DurationRegex = /P((([0-9]*\.?[0-9]*)Y)?(([0-9]*\.?[0-9]*)M)?(([0-9]*\.?[0-9]*)W)?(([0-9]*\.?[0-9]*)D)?)?(T(([0-9]*\.?[0-9]*)H)?(([0-9]*\.?[0-9]*)M)?(([0-9]*\.?[0-9]*)S)?)?/;

    var matches = durationString.match(iso8601DurationRegex);
    //console.log("parseISO8601Duration matches: " +matches);

    return {
        years: matches[3] === undefined ? 0 : parseInt(matches[3]),
        months: matches[5] === undefined ? 0 : parseInt(matches[5]),
        weeks: matches[7] === undefined ? 0 : parseInt(matches[7]),
        days: matches[9] === undefined ? 0 : parseInt(matches[9]),
        hours: matches[12] === undefined ? 0 : parseInt(matches[12]),
        minutes: matches[14] === undefined ? 0 : parseInt(matches[14]),
        seconds: matches[16] === undefined ? 0 : parseInt(matches[16])
    };
}

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
 * Checks for valid JSON and parses it. 
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

/*
 *
 */
function getItem(slots) {
    var propertyArray = Object.getOwnPropertyNames(data[0]);
    var value;

    for (var slot in slots)
    {
        if (slots[slot].value !== undefined)
        {
            value = slots[slot].value;
            for (var property in propertyArray)
            {
                var item = data.filter(x => x[propertyArray[property]].toString().toLowerCase() === slots[slot].value.toString().toLowerCase());
                if (item.length > 0)
                {
                    return item[0];
                }
            }
        }
    }
    return value;
}

/*
 * Creates Plotly figure data from a PFC CouchDB JSON.
 */
function mkPlotlyFigFromJson(res, graphTitle, xaxisTitle, yaxisTitle) {
    var obj = safelyParseJSON(res);
    var xPoints = [];
    var yPoints = [];

    for (var i = 0, len = obj.rows.length; i < len; i++) {
         var time = timeConverter(obj.rows[i].value.timestamp);
         xPoints.push(time);
         yPoints.push(obj.rows[i].value.value);
    }

    //console.log("xPoints: " +xPoints);
    //console.log("yPoints: " +yPoints);

    var trace = {
        x: xPoints,
        y: yPoints,
        type: "scatter"
    }

    var layout = {
       title: graphTitle,
       xaxis: {"title": xaxisTitle},
       yaxis: {"title": yaxisTitle}
    }

    var figure = {"data": [trace], "layout": layout};

    return figure;
}

/*
 * Creates Plotly figure data from PFC CouchDB CSV data.
 */
function mkPlotlyFigFromCsv(buf, graphTitle, xaxisTitle, yaxisTitle) {
    var str = buf.toString('utf8'); // Convert Buffer to string.
    //const deciBy = 1; // Decimate by this factor (1 = none). 

    try {
        var lines = str.split(/\r\n|\r|\n/); // Splits csv data at each new line.
        var xPoints = [];
        var yPoints = [];
        // Skip first (header) line and stop on last line.
        for (var l = 1, len = lines.length; l <= len - 1; l++) {
            // Decimate data to speed up plotly -- has no effect?
            //if (l % deciBy) continue;
            // Splits each line of csv by comma.
            var dataPoint = lines[l].split(',');
            // Ignore empty lines.
            if (!dataPoint[0]) continue;
            // Seperate data point into x and y arrays.
            var time = timeConverter(dataPoint[0]);
            // Time value. 
            xPoints.push(time);
            // Value of parameter vs time.
            yPoints.push(dataPoint[1]); 
        }
    } catch(e) {
        console.log("ERROR: mkPlotlyFigFromCsv: " +e);
    }

    //console.log("xPoints Len: " +xPoints.length);
    //console.log("yPoints Len: " +yPoints.length);

    var trace = {
        x: xPoints,
        y: yPoints,
        type: "scatter"
    }

    //console.log("title: "+graphTitle+" x: "+xaxisTitle+" y: "+yaxisTitle);

    var layout = {
       title: graphTitle,
       xaxis: {"title": xaxisTitle},
       yaxis: {"title": yaxisTitle}
    }

    var figure = {"data": [trace], "layout": layout};

    return figure;
}

/*
 * Debug - inspect and log object content.
 *
 */
function inspectLogObj(obj, depth = null) {
    const util = require("util");
    console.log(util.inspect(obj, {depth: depth}));
}
