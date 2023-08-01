// Example Google Scrips code to upload data to Google Sheets from Arduino/ESP8266
// based on previous work of StorageUnitB@gmail.com
// https://github.com/StorageB/Google-Sheets-Logging


// Enter Spreadsheet ID here
var SS = SpreadsheetApp.openById('11LWHlFNlHQVi8g2V9y9Qfw_WbB1ok5Yn8jtM2-29634');
var str = "";


function doPost(e) {

  var parsedData;
  var result = {};
  
  try { 
    parsedData = JSON.parse(e.postData.contents);
  } 
  catch(f){
    return ContentService.createTextOutput("Error in parsing request body: " + f.message);
  }
   
  if (parsedData !== undefined){
    var flag = parsedData.format;
    if (flag === undefined){
      flag = 0;
    }
    
    var sheet = SS.getSheetByName(parsedData.sheet_name); // sheet name to publish data to is specified in Arduino code
    var dataArr = parsedData.values.split(","); // creates an array of the values to publish 
         
    var date_now = Utilities.formatDate(new Date(), "America/Sao_Paulo", "yyyy/MM/dd"); // gets the current date
    var time_now = Utilities.formatDate(new Date(), "America/Sao_Paulo", "yyyy/MM/dd hh:mm:ss a"); // gets the current timestamp
    
    var value0 = dataArr [0]; // value0 from Arduino code
    var value1 = dataArr [1]; // value1 from Arduino code
    var value2 = dataArr [2]; // value2 from Arduino code
    var value3 = dataArr [3]; // value3 from Arduino code
    var value4 = dataArr [4]; // value4 from Arduino code
    var value5 = dataArr [5]; // value5 from Arduino code
    var value6 = dataArr [6]; // value6 from Arduino code
    var value7 = dataArr [7]; // value7 - obs values from Arduino code
    
    // read and execute command from the "payload_base" string specified in Arduino code
    switch (parsedData.command) {
      
      case "insert_row":
         
         sheet.insertRows(2); // insert full row directly below header text
         
         //var range = sheet.getRange("A2:D2");              // use this to insert cells just above the existing data instead of inserting an entire row
         //range.insertCells(SpreadsheetApp.Dimension.ROWS); // use this to insert cells just above the existing data instead of inserting an entire row
         
         sheet.getRange('A2').setValue(date_now); // publish current date to cell A2
         sheet.getRange('B2').setValue(time_now); // publish current time to cell B2
         sheet.getRange('C2').setValue(value0);   // publish value0 from Arduino code to cell C2
         sheet.getRange('D2').setValue(value1);   // publish value1 from Arduino code to cell D2
         sheet.getRange('E2').setValue(value2);   // publish value2 from Arduino code to cell E2
         sheet.getRange('F2').setValue(value3);   // publish value3 from Arduino code to cell F2
         sheet.getRange('G2').setValue(value4);   // publish value4 from Arduino code to cell G2
         sheet.getRange('H2').setValue(value5);   // publish value5 from Arduino code to cell H2
         sheet.getRange('I2').setValue(value6);   // publish value6 from Arduino code to cell I2
         sheet.getRange('J2').setValue(value7);   // publish value7 from Arduino code to cell J2

         str = "Success"; // string to return back to Arduino serial console
         SpreadsheetApp.flush();
         break;
         
      case "append_row":
         
         var publish_array = new Array(); // create a new array
         
         publish_array [0] = date_now; // add current date to position 0 in publish_array
         publish_array [1] = time_now; // add current time to position 1 in publish_array
         publish_array [2] = value0;   // add value0 from Arduino code to position 2 in publish_array
         publish_array [3] = value1;   // add value1 from Arduino code to position 3 in publish_array
         publish_array [4] = value2;   // add value2 from Arduino code to position 4 in publish_array
         publish_array [5] = value3;   // add value3 from Arduino code to position 5 in publish_array
         publish_array [6] = value4;   // add value4 from Arduino code to position 6 in publish_array
         publish_array [7] = value5;   // add value5 from Arduino code to position 7 in publish_array
         publish_array [8] = value6;   // add value6 from Arduino code to position 8 in publish_array
         publish_array [9] = value7;   // add value7 from Arduino code to position 9 in publish_array
        
         sheet.appendRow(publish_array); // publish data in publish_array after the last row of data in the sheet
         
         str = "Success"; // string to return back to Arduino serial console
         SpreadsheetApp.flush();
         break;     
 
    }
    
    return ContentService.createTextOutput(str);
  } // endif (parsedData !== undefined)
  
  else {
    return ContentService.createTextOutput("Error! Request body empty or in incorrect format.");
  }
}


function doGet(e) {
  var sheet = SS.getSheetByName("Parameters"); // sheet name where the parameters ares stored
  var targetA = sheet.getRange('param_targetA').getValue();
  var targetB = sheet.getRange('param_targetB').getValue();
  var delay   = sheet.getRange('param_delay').getValue();
  var max_duty_cycleA = sheet.getRange('param_maxdutyA').getValue();
  var max_duty_cycleB = sheet.getRange('param_maxdutyB').getValue();
  var Kp = sheet.getRange('param_Kp').getValue();
  var Kd = sheet.getRange('param_Kd').getValue();
  var Ki = sheet.getRange('param_Ki').getValue();

   var response = {
    "targetA": targetA,
    "targetB": targetB,
    "delay": delay,
    "max_duty_cycleA": max_duty_cycleA,
    "max_duty_cycleB": max_duty_cycleB,
    "Kp": Kp,
    "Kd": Kd,
    "Ki": Ki
  };
  
  return ContentService.createTextOutput(JSON.stringify(response)).setMimeType(ContentService.MimeType.JSON);
}
