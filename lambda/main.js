/**
 * Lambda function for OpenAg Personal Food Computer control and status triggred by Alexa.
 * 
 * For details see https://github.com/goruck.
 * 
 * Copyright (c) 2017 Lindo St. Angel
 */

// Read external files that identify host name and port of foodcomputer server. 
var fs = require('fs'),
    PORT = fs.readFileSync('./port.txt').toString().replace(/\n$/, ''), // Ignore last newline character
    HOST = fs.readFileSync('./host.txt').toString().replace(/\n$/, '');

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
    } else if ("GetStatus" === intentName) {
        getStatus(intent, session, callback);
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
             shared.buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

function getWelcomeResponse(callback) {
    // If we wanted to initialize the session to have some attributes we could add those here.
    var sessionAttributes = {};
    var cardTitle = "Welcome";
    var speechOutput = "to be defined";
    // If the user either does not reply to the welcome message, they will be prompted again.
    var repromptText = "to be defined";
    var shouldEndSession = false;

    callback(sessionAttributes,
             shared.buildSpeechletResponse(cardTitle, speechOutput, repromptText, shouldEndSession));
}

/*
 */
function startRecipe(intent, session, callback) {
    var cardTitle = intent.name;
    var sessionAttributes = {};
    var repromptText = "";
    var shouldEndSession = true;
    var speechOutput = "";

    var xhttp = new XMLHttpRequest();
    url = HOST + PORT + "/_openag/api/0.0.1/service/environments/environment_1/start_recipe";
    xhttp.open("POST", url, true);
    var data = '["test_lights"]';
    xhttp.send(data);
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        speechOutput = "started recipe";
      } else {
        speechOutput = "error, could not start recipe";
      }
      callback(sessionAttributes,
               shared.buildSpeechletResponse(intent.name, speechOutput, repromptText, shouldEndSession));
    }
}

// --------------- global variables and commonly used functions -----------------------

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
