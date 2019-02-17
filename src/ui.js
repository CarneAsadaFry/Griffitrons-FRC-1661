
// Define UI elements


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
        leftClawBar: document.getElementById('diagram-left-claw'),
        rightClawBar: document.getElementById('diagram-right-claw'),
        leftBallBar: document.getElementById('diagram-left-ball-intake'),
        rightBallBar: document.getElementById('diagram-right-ball-intake'),
        rotationBar: document.getElementById('diagram-rotation'),
        elevatorBar: document.getElementById('diagram-elevator'),
        leftDriveBar: document.getElementById('diagram-left-drive'),
        rightDriveBar: document.getElementById('diagram-right-drive'),

        leftClawBarVal: document.getElementById('diagram-left-claw-val'),
        rightClawBarVal: document.getElementById('diagram-right-claw-val'),
        leftBallBarVal: document.getElementById('diagram-left-ball-intake-val'),
        rightBallBarVal: document.getElementById('diagram-right-ball-intake-val'),
        rotationBarVal: document.getElementById('diagram-rotation-val'),
        elevatorBarVal: document.getElementById('diagram-elevator-val'),
        leftDriveBarVal: document.getElementById('diagram-left-drive-val'),
        rightDriveBarVal: document.getElementById('diagram-right-drive-val')
    },
    pid: {
        p: document.getElementById('p-val'),
        i: document.getElementById('i-val'),
        d: document.getElementById('d-val'),
        pCheck:document.getElementById('p-check'),
        iCheck:document.getElementById('i-check'),
        dCheck:document.getElementById('d-check'),
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
        temperature: document.getElementById('temperature'),

        voltageVal: document.getElementById('voltage-bar-val'),
        totaldrawVal: document.getElementById('total-draw-val'),
        drivedrawVal: document.getElementById('drive-train-val'),
        intakedrawVal: document.getElementById('intake-draw-val'),
        intakerotatedrawVal: document.getElementById('intake-pivot-draw-val'),
        elevatordrawVal: document.getElementById('elevator-draw-val'),
        velocityVal: document.getElementById('velocity-val'),
        accelerationVal: document.getElementById('acceleration-val'),
        temperatureVal: document.getElementById('temperature-val')
    },
    auto: {
        left: document.getElementById('field-bottom-left-square'),
        middle: document.getElementById('field-bottom-middle-square'),
        right: document.getElementById('field-bottom-right-square'),
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
        topLine1: document.getElementById('field-top-line-1'),
        bottomLine1: document.getElementById('field-bottom-line-1'),
        topLine2: document.getElementById('field-top-line-2'),
        bottomLine2: document.getElementById('field-bottom-line-2'),
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

}

NetworkTables.addKeyListener('/SmartDashboard/lDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.leftDriveBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.leftDriveBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/rDrive', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rightDriveBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rightDriveBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/leftClawIntake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.leftClawBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.leftClawBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/rightClawIntake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rightClawBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rightClawBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/rightBallIntake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rightBallBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rightBallBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/leftBallIntake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.leftBallBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.leftBallBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/rightBallIntake', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rightBallBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rightBallBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotate', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.rotationBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.rotationBar.value = 1 + (Math.floor(value * 100) / 100);
});
NetworkTables.addKeyListener('/SmartDashboard/elevator', (key, value) => {
    let num = Math.floor((value + 100) / 2);
    ui.robotDiagram.elevatorBarVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.robotDiagram.elevatorBar.value = 1 + (Math.floor(value * 100) / 100);
});


NetworkTables.addKeyListener('/SmartDashboard/voltage', (key, value) => {
    ui.power.voltageVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.voltage.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/totaldraw', (key, value) => {
    ui.power.totaldrawVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.totaldraw.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/drivedraw', (key, value) => {
    ui.power.drivedrawVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.drivedraw.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/intakedraw', (key, value) => {
    ui.power.intakedrawVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.intakedraw.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/intakerotatedraw', (key, value) => {
    ui.power.intakerotatedrawVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.intakerotatedraw.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/elevatordraw', (key, value) => {
    ui.power.elevatordrawVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.elevatordraw.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/velocity', (key, value) => {
    ui.power.velocityVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.velocity.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/acceleration', (key, value) => {
    ui.power.accelerationVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    ui.power.acceleration.value = value;
});
NetworkTables.addKeyListener('/SmartDashboard/temperature', (key, value) => {
    value = value * 9 / 5 + 32;
    ui.power.temperatureVal.innerHTML = (Math.floor(value * 100) / 100).toFixed(2);
    let percent = value / 120 * 100;
    ui.power.temperature.value = percent;
});

NetworkTables.addKeyListener('/SmartDashboard/p', (key, value) => {
    ui.pid.p.value = value;
    ui.pid.save.style.background = '#415359';
    ui.pid.pCheck.style.opacity = "0";
});
NetworkTables.addKeyListener('/SmartDashboard/i', (key, value) => {
    ui.pid.i.value = value;
    ui.pid.save.style.background = '#415359';
    ui.pid.iCheck.style.opacity = "0";
});
NetworkTables.addKeyListener('/SmartDashboard/d', (key, value) => {
    ui.pid.d.value = value;
    ui.pid.save.style.background = '#415359';
    ui.pid.dCheck.style.opacity = "0";
});
ui.pid.save.onclick = function() {
  NetworkTables.putValue('/SmartDashboard/p', ui.pid.p.value);
  NetworkTables.putValue('/SmartDashboard/i', ui.pid.i.value);
  NetworkTables.putValue('/SmartDashboard/d', ui.pid.d.value);
  ui.pid.save.style.background = 'linear-gradient(135deg, #f16000 0%,#ff2b51 100%)';
  ui.pid.pCheck.style.opacity = "1";
  ui.pid.iCheck.style.opacity = "1";
  ui.pid.dCheck.style.opacity = "1";
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
    var newValue = current.concat(value).concat("\n");
    NetworkTables.putValue('/SmartDashboard/consoleOutput', newValue);
    ui.jetson.console.innerHTML = NetworkTables.getValue('/SmartDashboard/consoleOutput');
});
NetworkTables.addKeyListener('/SmartDashboard/jetsonConnected', (key, value) => {
    if (value) {
        ui.jetson.isConnected.classList.add('color-icon');
    }
    else {
        ui.jetson.isConnected.classList.remove('color-icon');
    }
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

    if (value) {
        ui.field.leftRocket1.classList.remove('red');
        ui.field.rightRocket1.classList.remove('red');
        ui.field.leftRocket1.classList.add('swapBlue');
        ui.field.rightRocket1.classList.add('swapBlue');
        ui.field.leftRocket2.classList.remove('blue');
        ui.field.rightRocket2.classList.remove('blue');
        ui.field.leftRocket2.classList.add('swapRed');
        ui.field.rightRocket2.classList.add('swapRed');
        ui.field.topLine1.classList.remove('red-line');
        ui.field.topLine1.classList.add('blue-line');
        ui.field.bottomLine1.classList.remove('red-line');
        ui.field.bottomLine1.classList.add('blue-line');
        ui.field.topLine2.classList.remove('blue-line');
        ui.field.topLine2.classList.add('red-line');
        ui.field.bottomLine2.classList.remove('blue-line');
        ui.field.bottomLine2.classList.add('red-line');
        
    }
    else {
        ui.field.leftRocket1.classList.remove('blue');
        ui.field.rightRocket1.classList.remove('blue');
        ui.field.leftRocket1.classList.add('swapRed');
        ui.field.rightRocket1.classList.add('swapRed');
        ui.field.leftRocket2.classList.remove('red');
        ui.field.rightRocket2.classList.remove('red');
        ui.field.leftRocket2.classList.add('swapBlue');
        ui.field.rightRocket2.classList.add('swapBlue');
        ui.field.cargo1.classList.remove('blue-cargo')
        ui.field.cargo1.classList.add('swap-red-cargo')
        ui.field.cargo2.classList.remove('red-cargo')
        ui.field.cargo2.classList.add('swap-blue-cargo')
        ui.field.topLine1.classList.remove('blue-line');
        ui.field.topLine1.classList.add('red-line');
        ui.field.bottomLine1.classList.remove('blue-line');
        ui.field.bottomLine1.classList.add('red-line');
        ui.field.topLine2.classList.remove('red-line');
        ui.field.topLine2.classList.add('blue-line');
        ui.field.bottomLine2.classList.remove('red-line');
        ui.field.bottomLine2.classList.add('blue-line');
    }
   
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