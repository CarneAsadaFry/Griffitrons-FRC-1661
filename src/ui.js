// Define UI elements
let ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state').firstChild,
    gyro: {
        container: document.getElementById('gyroimg'),
        val: 0,
        offset: 0,
        visualVal: 0,
        arm: document.getElementById('arrowimg'),
        number: document.getElementById('gyronum'),
        reset: document.getElementById('gyroreset')
    },
    encoder: {
        flEnc: document.getElementById('flencval'),
        frEnc: document.getElementById('frencval'),
        blEnc: document.getElementById('blencval'),
        brEnc: document.getElementById('brencval'),
        reset: document.getElementById('encreset')
    },
    robotDiagram: {
        flDriveBar: document.getElementById('fldrivebar'),
        frDriveBar: document.getElementById('frdrivebar'),
        blDriveBar: document.getElementById('bldrivebar'),
        brDriveBar: document.getElementById('brdrivebar'),
        intakeBar: document.getElementById('intakebar'),
        intakerotateBar: document.getElementById('intakerotatebar'),
        elevatorBar: document.getElementById('elevatorbar')
    },
    pid: {
        p: document.getElementById('pfield'),
        i: document.getElementById('ifield'),
        d: document.getElementById('dfield'),
        save: document.getElementById('save')
    },
    power: {
        voltage: document.getElementById('voltagebar'),
        totaldraw: document.getElementById('totaldrawbar'),
        drivedraw: document.getElementById('drivedrawbar'),
        intakedraw: document.getElementById('intakedrawbar'),
        intakerotatedraw: document.getElementById('intakerotatedrawbar'),
        elevatordraw: document.getElementById('elevatordrawbar'),
        velocity: document.getElementById('velocitybar'),
        acceleration: document.getElementById('accelerationbar'),
        temperature: document.getElementById('temperaturebar')
    },
    auto: {
        left: document.getElementById('autoleft'),
        middle: document.getElementById('automiddle'),
        right: document.getElementById('autoright'),
        pantson: document.getElementById('pantson'),
        pantsoff: document.getElementById('pantsoff')
    },
    field: {
        scale1left: document.getElementById('scale1left'),
        scale1right: document.getElementById('scale1right'),
        scale2left: document.getElementById('scale2left'),
        scale2right: document.getElementById('scale2right'),
        scale3left: document.getElementById('scale3left'),
        scale3right: document.getElementById('scale3right'),
        teamcolor: document.getElementById('teamcolor'),
        opponentcolor: document.getElementById('opponentcolor'),
        leftcircle: document.getElementById('leftcircle'),
        middlecircle: document.getElementById('middlecircle'),
        rightcircle: document.getElementById('rightcircle'),
        leftfield: document.getElementById('leftimg'),
        leftpantsfield: document.getElementById('leftpantsimg'),
        middlefield: document.getElementById('middleimg'),
        middlepantsfield: document.getElementById('middlepantsimg'),
        rightfield: document.getElementById('rightimg'),
        rightpantsfield: document.getElementById('rightpantsimg')
    }
};

// Gyro rotation
let updateGyro = (key, value) => {
    ui.gyro.val = value;
    ui.gyro.visualVal = Math.floor(ui.gyro.val - ui.gyro.offset);
    ui.gyro.visualVal %= 360;
    if (ui.gyro.visualVal < 0) {
        ui.gyro.visualVal += 360;
    }
    ui.gyro.value = ui.gyro.visualVal;
    ui.gyro.arm.style.transform = `rotate(${ui.gyro.visualVal}deg)`;
    ui.gyro.number.innerHTML = ui.gyro.visualVal + 'º';
};
NetworkTables.addKeyListener('/SmartDashboard/gyro', updateGyro);
ui.gyro.reset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/gyroReset', true);
}

// Takes front left encoder value
NetworkTables.addKeyListener('/SmartDashboard/flEnc', (key, value) => {
    ui.encoder.flEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes front right encoder value
NetworkTables.addKeyListener('/SmartDashboard/frEnc', (key, value) => {
    ui.encoder.frEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes back left encoder value
NetworkTables.addKeyListener('/SmartDashboard/blEnc', (key, value) => {
    ui.encoder.blEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes back right encoder value
NetworkTables.addKeyListener('/SmartDashboard/brEnc', (key, value) => {
    ui.encoder.brEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
ui.encoder.reset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/encReset', true);
    NetworkTables.putValue('/SmartDashboard/flEnc', 0);
    NetworkTables.putValue('/SmartDashboard/frEnc', 0);
    NetworkTables.putValue('/SmartDashboard/blEnc', 0);
    NetworkTables.putValue('/SmartDashboard/brEnc', 0);
};

function onStart () {

}

NetworkTables.addKeyListener('/SmartDashboard/flDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.flDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.flDriveBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/frDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.frDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.frDriveBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/blDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.blDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.blDriveBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/brDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.brDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.brDriveBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/intake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.intakeBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.intakeBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotate', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.intakerotateBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.intakerotateBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/elevator', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.elevatorBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.elevatorBar.style.background = `linear-gradient(to right, red ${num}%, #A9A9A9 ${num}%)`;
});

NetworkTables.addKeyListener('/SmartDashboard/voltage', (key, value) => {
    ui.power.voltage.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'V';
    let percent = value / 13 * 100;
    ui.power.voltage.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/totaldraw', (key, value) => {
    ui.power.totaldraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 1037 * 100;
    ui.power.totaldraw.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/drivedraw', (key, value) => {
    ui.power.drivedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 532 * 100;
    ui.power.drivedraw.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/intakedraw', (key, value) => {
    ui.power.intakedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 106 * 100;
    ui.power.intakedraw.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotatedraw', (key, value) => {
    ui.power.intakerotatedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 133 * 100;
    ui.power.intakerotatedraw.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/elevatordraw', (key, value) => {
    ui.power.elevatordraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 266 * 100;
    ui.power.elevatordraw.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/velocity', (key, value) => {
    ui.power.velocity.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + ' m/s';
    let percent = value / 50 * 100;
    ui.power.velocity.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/acceleration', (key, value) => {
    ui.power.acceleration.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + ' m/s' + '2'.sup();
    let percent = value / 50 * 100;
    ui.power.acceleration.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});
NetworkTables.addKeyListener('/SmartDashboard/temperature', (key, value) => {
    value = value * 9 / 5 + 32;
    ui.power.temperature.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'ºF';
    let percent = value / 120 * 100;
    ui.power.temperature.style.background = `linear-gradient(to right, red ${percent}%, #A9A9A9 ${percent}%)`;
});

NetworkTables.addKeyListener('/SmartDashboard/p', (key, value) => {
    ui.pid.p.value = value;
    ui.pid.save.style.background = '#A9A9A9';
});
NetworkTables.addKeyListener('/SmartDashboard/i', (key, value) => {
    ui.pid.i.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/d', (key, value) => {
    ui.pid.d.value = value;
});
ui.pid.save.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/p', ui.pid.p.value);
  NetworkTables.putValue('/SmartDashboard/i', ui.pid.i.value);
  NetworkTables.putValue('/SmartDashboard/d', ui.pid.d.value);
  ui.pid.save.style.background = 'red';
}

NetworkTables.addKeyListener('/SmartDashboard/automode', (key, value) => {
   ui.auto.left.style.background = '#A9A9A9';
   ui.auto.middle.style.background = '#A9A9A9';
   ui.auto.right.style.background = '#A9A9A9';
   ui.field.leftcircle.style.background = '#222';
   ui.field.middlecircle.style.background = '#222';
   ui.field.rightcircle.style.background = '#222';
   ui.field.leftfield.style.opacity = 0;
   ui.field.leftpantsfield.style.opacity = 0;
   ui.field.middlefield.style.opacity = 0;
   ui.field.middlepantsfield.style.opacity = 0;
   ui.field.rightfield.style.opacity = 0;
   ui.field.rightpantsfield.style.opacity = 0;
   let pantsVal = NetworkTables.getValue('/SmartDashboard/pants');
   if(value === 0){
     ui.auto.left.style.background = 'red';
     ui.field.leftcircle.style.background = 'red';
     if(pantsVal) ui.field.leftpantsfield.style.opacity = 1;
     else ui.field.leftfield.style.opacity = 1;
   } else if(value === 1){
     ui.auto.middle.style.background = 'red';
     ui.field.middlecircle.style.background = 'red';
     if(pantsVal) ui.field.middlepantsfield.style.opacity = 1;
     else ui.field.middlefield.style.opacity = 1;
   } else {
     ui.auto.right.style.background = 'red';
     ui.field.rightcircle.style.background = 'red';
     if(pantsVal) ui.field.rightpantsfield.style.opacity = 1;
     else ui.field.rightfield.style.opacity = 1;
   }
});
ui.auto.left.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/automode', 0);
}
ui.auto.middle.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/automode', 1);
}
ui.auto.right.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/automode', 2);
}

NetworkTables.addKeyListener('/SmartDashboard/pants', (key, value) => {
   ui.auto.pantson.style.background = '#A9A9A9';
   ui.auto.pantsoff.style.background = '#A9A9A9';
   ui.field.leftfield.style.opacity = 0;
   ui.field.leftpantsfield.style.opacity = 0;
   ui.field.middlefield.style.opacity = 0;
   ui.field.middlepantsfield.style.opacity = 0;
   ui.field.rightfield.style.opacity = 0;
   ui.field.rightpantsfield.style.opacity = 0;
   let startPos = NetworkTables.getValue('/SmartDashboard/automode');
   if(value){
     ui.auto.pantson.style.background = 'red';
     if(startPos == 0) ui.field.leftpantsfield.style.opacity = 1;
     else if(startPos == 1) ui.field.middlepantsfield.style.opacity = 1;
     else ui.field.rightpantsfield.style.opacity = 1;
   } else {
     ui.auto.pantsoff.style.background = 'red';
     if(startPos == 0) ui.field.leftfield.style.opacity = 1;
     else if(startPos == 1) ui.field.middlefield.style.opacity = 1;
     else ui.field.rightfield.style.opacity = 1;
   }
});
ui.auto.pantson.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/pants', true);
}
ui.auto.pantsoff.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/pants', false);
}

NetworkTables.addKeyListener('/SmartDashboard/timer', (key, value) => {
    ui.timer.innerHTML = 'REMAINING TIME: ' + (value < 0 ? '0:00' : Math.floor(value / 60) + ':'
    + (value % 60 < 10 ? '0' : '') + Math.floor(value % 60 * 10) / 10 + (Math.floor(value % 60 * 10) / 10 === Math.floor(value % 60) ? '.0' : ''));
    if(value < 30 && !NetworkTables.getValue('/SmartDashboard/inauto')) {
      ui.timer.style.color = 'red';
    } else {
      ui.timer.style.color = 'white';
    }
});

NetworkTables.addKeyListener('/SmartDashboard/isred', (key, value) => {
    if(value) {
      ui.field.teamcolor.style.background = 'red';
      ui.field.opponentcolor.style.background = 'blue';
    }else {
      ui.field.teamcolor.style.background = 'blue';
      ui.field.opponentcolor.style.background = 'red';
    }
    ui.field.scale1left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale1left')) ? 'red' : 'blue';
    ui.field.scale1right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale1left')) ? 'blue' : 'red';
    ui.field.scale2left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale2left')) ? 'red' : 'blue';
    ui.field.scale2right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale2left')) ? 'blue' : 'red';
    ui.field.scale3left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale3left')) ? 'red' : 'blue';
    ui.field.scale3right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale3left')) ? 'blue' : 'red';
});
NetworkTables.addKeyListener('/SmartDashboard/scale1left', (key, value) => {
    ui.field.scale1left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
    ui.field.scale1right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
});
NetworkTables.addKeyListener('/SmartDashboard/scale2left', (key, value) => {
    ui.field.scale2left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
    ui.field.scale2right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
});
NetworkTables.addKeyListener('/SmartDashboard/scale3left', (key, value) => {
    ui.field.scale3left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
    ui.field.scale3right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
});

addEventListener('error',(ev)=>{
    ipc.send('windowError',ev)
})
