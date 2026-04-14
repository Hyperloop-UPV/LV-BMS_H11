/*
 * Copyright 2016 - 2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Associative array with mapping of subscribed variables to element IDs.
 */
var SubscribedVariables = {
  "stackVoltageUV" : "MEAS_STACK",
  "cell1VoltageUV" : "MEAS_CELL1",
  "cell2VoltageUV" : "MEAS_CELL2",
  "cell3VoltageUV" : "MEAS_CELL3",
  "cell4VoltageUV" : "MEAS_CELL4",
  "cell5VoltageUV" : "MEAS_CELL5",
  "cell6VoltageUV" : "MEAS_CELL6",
  "cell7VoltageUV" : "MEAS_CELL7",
  "cell8VoltageUV" : "MEAS_CELL8",
  "cell9VoltageUV" : "MEAS_CELL9",
  "cell10VoltageUV" : "MEAS_CELL10",
  "cell11VoltageUV" : "MEAS_CELL11",
  "cell12VoltageUV" : "MEAS_CELL12",
  "cell13VoltageUV" : "MEAS_CELL13",
  "cell14VoltageUV" : "MEAS_CELL14",
  "an0TempDegC" : "MEAS_AN0",
  "an1TempDegC" : "MEAS_AN1",
  "an2TempDegC" : "MEAS_AN2",
  "an3TempDegC" : "MEAS_AN3",
  "an4TempDegC" : "MEAS_AN4",
  "an5TempDegC" : "MEAS_AN5",
  "an6TempDegC" : "MEAS_AN6",
  "icTempDegC" : "MEAS_IC_TEMP",
  "vBGADC1AVoltageUV" : "MEAS_VBG_DIAG_ADC1A",
  "vBGADC1BVoltageUV" : "MEAS_VBG_DIAG_ADC1B",
  "faultStatus1" : "bcc-fault1-status-tab",
  "faultStatus2" : "bcc-fault2-status-tab",
  "faultStatus3" : "bcc-fault3-status-tab"
}

var cbOn = 0;

function PageInit()
{ 
  var Result;
  
  // subscribes variables watched by FreeMaster
  $.each(SubscribedVariables, function(key, value) {
    fmstr.SubscribeVariable(key, 1000, Result);
  });
  
  // attaches listener to change event
  fmstr.attachEvent("OnVariableChanged", OnVariableChanged, true);
}

function OnVariableChanged(bsVarName, vSubscriptionId)
{
  fmstr.ReadVariable(bsVarName);
  
  if ((bsVarName == "faultStatus1") | (bsVarName == "faultStatus2") | (bsVarName == "faultStatus3")) {
    UpdateFaultStatus(SubscribedVariables[bsVarName], fmstr.LastVariable_vValue);
  }
  else if ((bsVarName == "an0TempDegC") | (bsVarName == "an1TempDegC") | (bsVarName == "an2TempDegC") | (bsVarName == "an3TempDegC") | (bsVarName == "an4TempDegC") | (bsVarName == "an5TempDegC") | (bsVarName == "an6TempDegC") | (bsVarName == "icTempDegC")) {
    measValScaled = fmstr.LastVariable_vValue / 10;
    UpdateMeasurement(SubscribedVariables[bsVarName], measValScaled.toFixed(1));  
  }
  else {
    measValScaled = fmstr.LastVariable_vValue / 1000;
    UpdateMeasurement(SubscribedVariables[bsVarName], measValScaled.toFixed(1));
  }
}

function UpdateMeasurement(elemID, Value) {
  $("#" + elemID).html(Value);
}

function UpdateFaultStatus(elemID, Value) {
  $("#" + elemID).find("img").each(function(index){
    if (Value & (1 << index)) {
      $(this).attr('src', 'images/LED/redLed.png')
    }
    else {
      $(this).attr('src', 'images/LED/greenLed.png')
    }
  });  
}

/**
 * Sends command using FreeMASTER ActiveX object.
 * Handles possible exceptions.
 */
function SendCmd(Cmd) {
  var CmdRes;
  //console.log(Cmd);
  try {
    if (!fmstr.SendCommand(Cmd, CmdRes)) {
      alert("Communication timeout expired");
    }
  } catch (e) {
    //console.log(e);
  }
}

/**
 * Sets device mode.
 */
function SetMode(Mode) {  
  SendCmd(Mode == "sleep" ? "Sleep()" : "WakeUp()");
}

/**
 * Enables or disables cell balancing.
 */
function SetCB(EnDis) {
  cbOn = EnDis;
  SendCmd("EnableCB(" + EnDis + ")");
}

/**
 * Clears all possible faults.
 */
function ClearFaults() {
  SendCmd("ClearFaults()");
}

/**
 * Resets BCC device via SPI.
 */
function SoftwareReset() {  
  SendCmd("SoftwareReset()");
}

/**
 * Sends FreeMASTER command to update cell balancing timeout.
 */
function UpdateCBTimeout(Timeout) {  
  SendCmd("UpdateCBTimeout(" + Timeout + ")");
}

/**
 * Sends FreeMASTER command to update selected threshold.
 */
function UpdateThreshold(ThreshSel, ThreshVal) {  
  SendCmd("UpdateThreshold(" + ThreshSel + ", " + ThreshVal +")");
}

/**
 * Adds styleTable (using jQuery CSS framework) function to jQuery object.
 */
(function ($) {
  $.fn.styleTable = function (options) {
  var defaults = {
    css: 'styleTable'
  };
  options = $.extend(defaults, options);

  return this.each(function () {
    $this = $(this);
    $this.addClass(options.css);

    $this.on('mouseover mouseout', 'tbody tr', function (event) {
      $(this).children("td").toggleClass("ui-state-hover", event.type == 'mouseover');
    });

    $this.find("th").addClass("ui-state-default");
    $this.find("td").addClass("ui-widget-content");

    $this.find("th").addClass("first");
    $this.find("td").addClass("first");
    $this.find("th:first-child").removeClass("first");
    $this.find("td:first-child").removeClass("first");
    });

  };
})(jQuery);

/**
 * Adds .error() and .highlight() functions to jQuery object.
 * Uses jQuery UI CSS framework to style text in paragraphs.
 */
function errorHighlight(e, type, icon) {
  if (!icon) {
    if (type === 'highlight') {
      icon = 'ui-icon-info';
    } else {
      icon = 'ui-icon-alert';
    }
  }
  return e.each(function () {
    $(this).addClass('ui-widget');
    var h = '<div class="ui-state-' + type + ' ui-corner-all" style="padding:0 .7em;">';
    h += '<p>';
    h += '<span class="ui-icon ' + icon + '" style="float:left;margin-right: .3em;"></span>';
    h += $(this).text();
    h += '</p>';
    h += '</div>';

    $(this).html(h);
  });
}

//error dialog
(function ($) {
  $.fn.error = function () {
    errorHighlight(this, 'error');
  };
})(jQuery);

//highlight (alert) dialog
(function ($) {
  $.fn.highlight = function () {
    errorHighlight(this, 'highlight');
  };
})(jQuery);

/**
 * Disables all UI control for CB.
 */
function DisableCBControl() {
  $("#bcc-controls-cb-timeout").slider("disable");
  document.getElementById("cb-1").disabled = true;
  document.getElementById("cb-2").disabled = true;
  document.getElementById("cb-3").disabled = true;
  document.getElementById("cb-4").disabled = true;
  document.getElementById("cb-5").disabled = true;
  document.getElementById("cb-6").disabled = true;
  document.getElementById("cb-7").disabled = true;
  document.getElementById("cb-8").disabled = true;
  document.getElementById("cb-9").disabled = true;
  document.getElementById("cb-10").disabled = true;
  document.getElementById("cb-11").disabled = true;
  document.getElementById("cb-12").disabled = true;
  document.getElementById("cb-13").disabled = true;
  document.getElementById("cb-14").disabled = true;
}

/**
 * Enable all UI control for CB.
 */
function EnableCBControl() {
  $("#bcc-controls-cb-timeout").slider("enable");
  document.getElementById("cb-1").disabled = false;
  document.getElementById("cb-2").disabled = false;
  document.getElementById("cb-3").disabled = false;
  document.getElementById("cb-4").disabled = false;
  document.getElementById("cb-5").disabled = false;
  document.getElementById("cb-6").disabled = false;
  document.getElementById("cb-7").disabled = false;
  document.getElementById("cb-8").disabled = false;
  document.getElementById("cb-9").disabled = false;
  document.getElementById("cb-10").disabled = false;
  document.getElementById("cb-11").disabled = false;
  document.getElementById("cb-12").disabled = false;
  document.getElementById("cb-13").disabled = false;
  document.getElementById("cb-14").disabled = false;
}

/**
 * Disables all control items (SPI is not functional during sleep mode).
 */
function DisableControl() {
  $("#bcc-controls-reset").button("disable");
  $("#bcc-controls-clear").button("disable");
  $("#bcc-controls-cb").buttonset("disable");
  $("#bcc-thresholds-gpiox-otut").slider("disable");
  $("#bcc-thresholds-ctx-ovuv").slider("disable");
  DisableCBControl();
}

/**
 * Change the individual CB settings.
 */
function CBxChanged(checkboxEl, id) {
  if (checkboxEl.checked) {
    SendCmd("SetCBIndividually(" + id + ", 1)");
  } else {
    SendCmd("SetCBIndividually(" + id + ", 0)");
  }
}

/**
 * Enables all control items.
 */
function EnableControl() {
  $("#bcc-controls-reset").button("enable");
  $("#bcc-controls-clear").button("enable");
  $("#bcc-controls-cb").buttonset("enable");
  $("#bcc-thresholds-gpiox-otut").slider("enable");
  $("#bcc-thresholds-ctx-ovuv").slider("enable");
  
  if (cbOn == 0) {
    EnableCBControl();
  }
}

/**
 * Initializes widgets after DOM is loaded.
 */
$(document).ready(function() 
{
  /* Enables tooltips using 'title' attribute on DOM elements. */
  $(document).tooltip();
  
  document.getElementById("cb-1").disabled = false;
  document.getElementById("cb-2").disabled = false;
  document.getElementById("cb-3").disabled = false;
  document.getElementById("cb-4").disabled = false;
  document.getElementById("cb-5").disabled = false;
  document.getElementById("cb-6").disabled = false;
  document.getElementById("cb-7").disabled = false;
  document.getElementById("cb-8").disabled = false;
  document.getElementById("cb-9").disabled = false;
  document.getElementById("cb-10").disabled = false;
  document.getElementById("cb-11").disabled = false;
  document.getElementById("cb-12").disabled = false;
  document.getElementById("cb-13").disabled = false;
  document.getElementById("cb-14").disabled = false;

  $("#bcc-controls-mode").buttonset();
  $("#mode-sleep").click(function(event, ui) {
    event.preventDefault();
    SetMode('sleep');
    DisableControl();
  });
  $("#mode-normal").click(function(event, ui) {
    event.preventDefault();
    SetMode('normal');
    EnableControl();
  });  
  
  $("#bcc-controls-cb").buttonset();
  $("#bcc-controls-cb-timeout").slider({
    disabled: false,
    orientation: "horizontal",
    range: "min",
    min: 0,
    max: 511,
    value: 1,
    slide: function(event, ui) {
      if (parseInt(ui.value) == 0) {
        $("#bcc-controls-cb-timeout-amount").val('0.5');
      }
      else {
        $("#bcc-controls-cb-timeout-amount").val(ui.value);
      }
    },
    stop: function(event, ui) {
      UpdateCBTimeout(parseInt(ui.value));
    }
  });
  $("#bcc-controls-cb-timeout-amount").val($("#bcc-controls-cb-timeout").slider("value"));
  
  $("#cb-on").click(function(event, ui) {
    event.preventDefault();
    DisableCBControl();
    SetCB(1);
  });
  $("#cb-off").click(function(event, ui) {
    event.preventDefault();
    EnableCBControl();
    SetCB(0);
  });
    
  $("#bcc-controls-clear")
    .button()
    .click(function(event) {
      event.preventDefault();
      ClearFaults();
    });
    
  $("#bcc-controls-reset")
    .button()
    .click(function(event) {
      event.preventDefault();
      SoftwareReset();

      cbOn = 0;
      document.getElementById("cb-off").checked = true;
      $("#bcc-controls-cb").buttonset("refresh");
      EnableCBControl();
    });
  
  $("#monitoring-tabs").tabs();
  
  $("#bcc-measurements-tab").styleTable();
  $("#bcc-fault1-status-tab").styleTable();
  $("#bcc-fault2-status-tab").styleTable();
  $("#bcc-fault3-status-tab").styleTable();
  
  $("#bcc-thresholds-gpiox-otut").slider({
    range: true,
    min: 0,
    max: 5000,
    values: [1160, 3820],
    slide: function(event, ui) {
      $("#bcc-thresholds-gpiox-otut-amount").val(ui.values[0] + " : " + ui.values[1]);
    },
    stop: function(event, ui) {
      UpdateThreshold(0, parseInt(ui.values[0]));
      UpdateThreshold(1, parseInt(ui.values[1]));
    }
  });
  
  $("#bcc-thresholds-gpiox-otut-amount").val($("#bcc-thresholds-gpiox-otut").slider("values", 0) +
    " : " + $("#bcc-thresholds-gpiox-otut").slider("values", 1 ));
    
  $("#bcc-thresholds-ctx-ovuv").slider({
    range: true,
    min: 0,
    max: 5000,
    values: [2800, 4300],
    slide: function(event, ui) {
      $("#bcc-thresholds-ctx-ovuv-amount").val(ui.values[0] + " : " + ui.values[1]);
    },
    stop: function(event, ui) {
      UpdateThreshold(2, parseInt(ui.values[1]));
      UpdateThreshold(3, parseInt(ui.values[0]));
    }
  });
  $("#bcc-thresholds-ctx-ovuv-amount").val($("#bcc-thresholds-ctx-ovuv").slider("values", 0) +
    " : " + $("#bcc-thresholds-ctx-ovuv").slider("values", 1 ));
  
  /* Default values of status information. */
  UpdateFaultStatus(SubscribedVariables['faultStatus1'], 0x0000);
  UpdateFaultStatus(SubscribedVariables['faultStatus2'], 0x0000);
  UpdateFaultStatus(SubscribedVariables['faultStatus3'], 0x0000);
});
