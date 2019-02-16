import { isUndefined } from "util";

// Define UI elements

//Comment out all stuff idk what to do with.



let ui = {
    timer: document.getElementById('timer'),
    robotState: document.getElementById('robot-state').firstChild,
    gyro: {
        container: document.getElementById('orientation-grid'),
        val: 0,
        offset: 0,
        visualVal: 0,
        arm: document.getElementById('arrow'),
        number: document.getElementById('orientation-value'),
        reset: document.getElementById('reset-orientation')
    },
    encoder: {
        lEnc: document.getElementById('left-drive-encoder-value'),
        rEnc: document.getElementById('right-drive-encoder-value'),
        intakeEnc: document.getElementById('intake-encoder-value'),
        rotationEnc: document.getElementById('rotation-encoder-value'),
        elevatorEnc: document.getElementById('elevator-encoder-value'),
        
        lEncReset: document.getElementById('left-drive-reset'),
        rEncReset: document.getElementById('right-drive-reset'),
        intakeEncReset: document.getElementById('intake-reset'),
        rotationEncReset: document.getElementById('rotation-reset'),
        elevatorEncReset: document.getElementById('elevator-reset')
    
    },
    robotDiagram: {
        leftClawBar: document.getElementById('diagram-left-claw-value'),
        rightClawBar: document.getElementById('diagram-right-claw-value'),
        leftBallBar: document.getElementById('diagram-left-ball-intake-value'),
        rightBallBar: document.getElementById('diagram-right-ball-intake-value'),
        rotationBar: document.getElementById('diagram-rotation-value'),
        elevatorBar: document.getElementById('diagram-elevator-value'),
        leftDriveBar: document.getElementById('diagram-left-drive-value'),
        rightDriveBar: document.getElementById('diagram-right-drive-value')
    },
    pid: {
        p: document.getElementById('p-val'),
        i: document.getElementById('i-val'),
        d: document.getElementById('d-val'),
        save: document.getElementById('pid-button')
    },
    power: {
        voltage: document.getElementById('voltage-bar'),
        totaldraw: document.getElementById('total-draw'),
        drivedraw: document.getElementById('drive-train'),
        intakedraw: document.getElementById('intake-draw'),
        intakerotatedraw: document.getElementById('intake-pivot-draw'),
        elevatordraw: document.getElementById('elevator-draw'),
        velocity: document.getElementById('velocity'),
        acceleration: document.getElementById('acceleration'),
        temperature: document.getElementById('temperature')
    },
    auto: {
        left: document.getElementById('left-button'),
        middle: document.getElementById('middle-button'),
        right: document.getElementById('right-button'),
    },
    jetson: {
        console: document.getElementById('console-interior'),
        isConnected: document.getElementById('light')
    },
    field: {
        topLeftSquare: document.getElementById('field-top-left-square'),
        topMiddleSquare: document.getElementById('field-top-middle-square'),
        topRightSquare: document.getElementById('field-top-right-square'),
        bottomLeftSquare: document.getElementById('field-bottom-left-square'),
        bottomMiddleSquare: document.getElementById('field-bottom-middle-square'),
        bottomRightSquare: document.getElementById('field-bottom-right-square'),
        topLine1: document.getElementById('field-top-line-1'),
        bottomLine1: document.getElementById('field-bottom-line-1'),
        topLine2: document.getElementById('field-top-line-2'),
        bottomLine2: document.getElementById('field-bottom-line-2'),
        leftRocket1: document.getElementById('field-left-rocket-1'),
        rightRocket1: document.getElementById('field-right-rocket-1'),
        leftRocket2: document.getElementById('field-left-rocket-2'),
        rightRocket2: document.getElementById('field-right-rocket-2'),
        cargo1: document.getElementById('field-cargo-1'),
        cargo2: document.getElementById('field-cargo-2'),
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

// Takes left encoder value
NetworkTables.addKeyListener('/SmartDashboard/lEnc', (key, value) => {
    ui.encoder.lEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes right encoder value
NetworkTables.addKeyListener('/SmartDashboard/rEnc', (key, value) => {
    ui.encoder.rEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes intake encoder value
NetworkTables.addKeyListener('/SmartDashboard/intakeEnc', (key, value) => {
    ui.encoder.intakeEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes rotation encoder value
NetworkTables.addKeyListener('/SmartDashboard/rotationEnc', (key, value) => {
    ui.encoder.rotationEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});
// Takes elevator encoder value
NetworkTables.addKeyListener('/SmartDashboard/elevatorEnc', (key, value) => {
    ui.encoder.elevatorEnc.innerHTML = (Math.floor(value * 10) / 10).toFixed(1);
});

ui.encoder.lEncReset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/lEncReset', true);
    NetworkTables.putValue('/SmartDashboard/lEnc', 0);
};
ui.encoder.rEncReset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/rEncReset', true);
    NetworkTables.putValue('/SmartDashboard/rEnc', 0);
};
ui.encoder.intakeEncReset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/intakeEncReset', true);
    NetworkTables.putValue('/SmartDashboard/intakeEnc', 0);
};
ui.encoder.rotationEncReset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/rotationEncReset', true);
    NetworkTables.putValue('/SmartDashboard/rotationEnc', 0);
};
ui.encoder.elevatorEncReset.onclick = function() {
    NetworkTables.putValue('/SmartDashboard/elevatorEncReset', true);
    NetworkTables.putValue('/SmartDashboard/elevatorEnc', 0);
};

function onStart () {
    // ui.robotDiagram.frDriveBar.innerHTML = 77;
    // var x = document.getElementById('temperature-val');
    var x = document.getElementById('temperature-val');
    var y = document.getElementById('temperature');
    var arrow = document.getElementById('arrow');
   
    // document.write(x);
    var myVar = setInterval(myTimer, 10);
            function myTimer() {
                var d = new Date();
                q = d.getSeconds()
                p = d.getMilliseconds() / 10;
                y.value = p;
                x.innerHTML = p;
                arrow.style.transform = ("rotate(" + (q * 6) + "deg)");
            }
    
}

NetworkTables.addKeyListener('/SmartDashboard/lDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.leftDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.leftDriveBar.value = num;
});
NetworkTables.addKeyListener('/SmartDashboard/rDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rightDriveBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rightDriveBar.value = num;
});
NetworkTables.addKeyListener('/SmartDashboard/intake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.intakeBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.intakeBar.value = num;
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotate', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.intakerotateBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.intakerotateBar.value = num;
});
NetworkTables.addKeyListener('/SmartDashboard/elevator', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.elevatorBar.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.elevatorBar.value = num;
});

NetworkTables.addKeyListener('/SmartDashboard/voltage', (key, value) => {
    ui.power.voltage.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'V';
    let percent = value / 13 * 100;
    ui.power.voltage.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/totaldraw', (key, value) => {
    ui.power.totaldraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 1037 * 100;
    ui.power.totaldraw.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/drivedraw', (key, value) => {
    ui.power.drivedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 532 * 100;
    ui.power.drivedraw.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/intakedraw', (key, value) => {
    ui.power.intakedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 106 * 100;
    ui.power.intakedraw.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotatedraw', (key, value) => {
    ui.power.intakerotatedraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 133 * 100;
    ui.power.intakerotatedraw.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/elevatordraw', (key, value) => {
    ui.power.elevatordraw.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'A';
    let percent = value / 266 * 100;
    ui.power.elevatordraw.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/velocity', (key, value) => {
    ui.power.velocity.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + ' m/s';
    let percent = value / 50 * 100;
    ui.power.velocity.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/acceleration', (key, value) => {
    ui.power.acceleration.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + ' m/s' + '2'.sup();
    let percent = value / 50 * 100;
    ui.power.acceleration.value = percent;
});
NetworkTables.addKeyListener('/SmartDashboard/temperature', (key, value) => {
    value = value * 9 / 5 + 32;
    ui.power.temperature.innerHTML = (Math.floor(value * 100) / 100).toFixed(2) + 'ºF';
    let percent = value / 120 * 100;
    ui.power.temperature.value = percent;
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

    //Replace this with new mechanism.


//    ui.auto.left.style.background = '#A9A9A9';
//    ui.auto.middle.style.background = '#A9A9A9';
//    ui.auto.right.style.background = '#A9A9A9';
//    ui.field.leftcircle.style.background = '#222';
//    ui.field.middlecircle.style.background = '#222';
//    ui.field.rightcircle.style.background = '#222';
//    ui.field.leftfield.style.opacity = 0;
//    ui.field.leftpantsfield.style.opacity = 0;
//    ui.field.middlefield.style.opacity = 0;
//    ui.field.middlepantsfield.style.opacity = 0;
//    ui.field.rightfield.style.opacity = 0;
//    ui.field.rightpantsfield.style.opacity = 0;
//    let pantsVal = NetworkTables.getValue('/SmartDashboard/pants');
//    if(value === 0){
//      ui.auto.left.style.background = 'red';
//      ui.field.leftcircle.style.background = 'red';
//      if(pantsVal) ui.field.leftpantsfield.style.opacity = 1;
//      else ui.field.leftfield.style.opacity = 1;
//    } else if(value === 1){
//      ui.auto.middle.style.background = 'red';
//      ui.field.middlecircle.style.background = 'red';
//      if(pantsVal) ui.field.middlepantsfield.style.opacity = 1;
//      else ui.field.middlefield.style.opacity = 1;
//    } else {
//      ui.auto.right.style.background = 'red';
//      ui.field.rightcircle.style.background = 'red';
//      if(pantsVal) ui.field.rightpantsfield.style.opacity = 1;
//      else ui.field.rightfield.style.opacity = 1;
//    }
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

// Not yet sure how to format these
NetworkTables.addKeyListener('/SmartDashboard/consoleOutput', (key, value) => {
    var current = jetson.console.getValue;
    var newString = current.concat(value);
    jetson.console.putValue(newString);
});
NetworkTables.addKeyListener('/SmartDashboard/jetsonConnected', (key, value) => {
    ui.jetson.isConnected.classList.add('color-icon');
});

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

    //Replace with new field color switch

    // if(value) {
    //   ui.field.teamcolor.style.background = 'red';
    //   ui.field.opponentcolor.style.background = 'blue';
    // }else {
    //   ui.field.teamcolor.style.background = 'blue';
    //   ui.field.opponentcolor.style.background = 'red';
    // }
    // ui.field.scale1left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale1left')) ? 'red' : 'blue';
    // ui.field.scale1right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale1left')) ? 'blue' : 'red';
    // ui.field.scale2left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale2left')) ? 'red' : 'blue';
    // ui.field.scale2right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale2left')) ? 'blue' : 'red';
    // ui.field.scale3left.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale3left')) ? 'red' : 'blue';
    // ui.field.scale3right.style.background = (value == NetworkTables.getValue('/SmartDashboard/scale3left')) ? 'blue' : 'red';
});
// NetworkTables.addKeyListener('/SmartDashboard/scale1left', (key, value) => {
//     ui.field.scale1left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
//     ui.field.scale1right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
// });
// NetworkTables.addKeyListener('/SmartDashboard/scale2left', (key, value) => {
//     ui.field.scale2left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
//     ui.field.scale2right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
// });
// NetworkTables.addKeyListener('/SmartDashboard/scale3left', (key, value) => {
//     ui.field.scale3left.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'red' : 'blue';
//     ui.field.scale3right.style.background = (value == NetworkTables.getValue('/SmartDashboard/isred')) ? 'blue' : 'red';
// });

addEventListener('error',(ev)=>{
    ipc.send('windowError',ev)
})
