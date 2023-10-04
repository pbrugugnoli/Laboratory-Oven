// Example Google Scrips code to upload data to Google Sheets from Arduino/ESP8266
// based on previous work of StorageUnitB@gmail.com
// https://github.com/StorageB/Google-Sheets-Logging


// Enter Spreadsheet ID here
var SS = SpreadsheetApp.openById('11LWHlFNlHQVi8g2V9y9Qfw_WbB1ok5Yn8jtM2-29634');
var str = "";

function doPost(e) {

  var parsedData;
  
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

    var time_stamp = Utilities.formatDate(new Date(), "America/Sao_Paulo", "yyyy/MM/dd hh:mm:ss a"); // gets the current timestamp
    var values = time_stamp + "," + parsedData.values;
    var dataArr = values.split(","); // creates an array of the values to publish 
          
    // read and execute command from the "payload_base" string specified in Arduino code
    switch (parsedData.command) {
      
      case "insert_row":         
         sheet.insertRows(2); // insert full row directly below header text
         sheet.getRange(2,1,1,dataArr.length).setValues([dataArr]);        
         sheet.getRange(3,1,1,dataArr.length).copyTo(sheet.getRange(2,1,1,dataArr.length), {formatOnly:true});
         str = "Success"; // string to return back to Arduino serial console
         SpreadsheetApp.flush();
         break;
         
      case "append_row":                
         sheet.appendRow(dataArr); // publish data in publish_array after the last row of data in the sheet         
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
  var sheet = SS.getSheetByName("Parameters"); // sheet name where the parameters are stored
  var parameters = sheet.getRange('param_list').getValues();
  var numRows = sheet.getRange('param_list').getNumRows();
  var response = {};

  for (var i = 0; i < numRows; i++) {
    response[parameters[i][0]] = parameters[i][1]
  }
  
  console.log(response);
  return ContentService.createTextOutput(JSON.stringify(response)).setMimeType(ContentService.MimeType.JSON);
}