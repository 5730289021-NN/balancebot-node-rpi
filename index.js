const Cylon = require('cylon');
const piblaster = require('pi-blaster.js');

//Offset and Scaling
//const fixAccOffset = (x, os_i) => x - accOffset[os_i];
//const fixGyroOffset = (x, os_i) => x - gyroOffset[os_i];
//const fixScaling = x => (x / 16384.0);

//Variable offset settings
//const accOffset = [250, 16300, -300];
//const gyroOffset = [10, 200, -140];

//Pin Configurations
const leftForwardPin = 23;
const leftBackwardPin = 24;
const rightForwardPin = 17;
const rightBackwardPin = 27;

const leftControlPin = 7;
const rightControlPin = 8;


//PID Constants (let is for adjustable PID);
let Kp = 0.0150;
let Ki = 0.000;
let Kd = 0.0;
let PID = 0;

let samplingTime = 0.05;
let oldErrorAngle = 0;
let sumErrorAngle = 0;
let motorOut = 0;
let a = 0.85;

let measuredAngle = 0;
let desiredAngle = 0;
let errorAngle = 0;

let deactivate = false;
let deactivateThreshold = 0.4;

const limitMode = false;

Cylon.robot({
    connections: {
        raspi: {
            adaptor: 'raspi'
        }
    },
    devices: {
        mpu6050: {
            driver: 'mpu6050'
        },
    },

    work: function (my) {
        every((samplingTime).seconds(), function () {
            console.log('Into Every');
            my.mpu6050.getMotionAndTemp(function (err, data) {
                //let acc = data.a.map(fixAccOffset).map(fixScaling);
                //let gyro = data.g.map(fixGyroOffset).map(fixScaling);
                //Desired Angle
                desiredAngle = 0;
                //Measured Angle
                let accAngle = (Math.atan2(data.a[1], -data.a[2]) * 360 / Math.PI) - 180;
                let gyroRate = data.g[0] * 250 / 32767;
                let gyroAngle = gyroRate * samplingTime;
                measuredAngle = a * (measuredAngle + gyroAngle)  + (1-a) * accAngle;

                //PID Components
                oldErrorAngle = errorAngle;
                errorAngle = desiredAngle - measuredAngle;
                sumErrorAngle += errorAngle;
                PID = Kp * errorAngle +  Kd * (errorAngle - oldErrorAngle) / samplingTime + Ki * sumErrorAngle;
                //Plant & Process
                motorOut = PID;
                
                if(limitMode) {
                    if(Math.abs(motorOut) > deactivateThreshold)
                    {
                        if(motorOut > 0) {
                            motorOut = deactivateThreshold;
                        }else {
                            motorOut = -deactivateThreshold;
                        }
                    }
                    console.log('Limit Mode Active');
                }
                
                if(Math.abs(motorOut) > deactivateThreshold || deactivate){
                    piblaster.setPwm(leftForwardPin, 0);
                    piblaster.setPwm(rightForwardPin, 0);
                    piblaster.setPwm(leftBackwardPin, 0);
                    piblaster.setPwm(rightBackwardPin, 0);
                    deactivate = true;
                    console.log('Deactivated because motor out: ', motorOut);
                } else{
                    if(motorOut >= 0){
                        piblaster.setPwm(leftForwardPin, 1);
                        piblaster.setPwm(leftBackwardPin, 0);
                        piblaster.setPwm(rightForwardPin, 1);
                        piblaster.setPwm(rightBackwardPin, 0);
                        piblaster.setPwm(leftControlPin, motorOut)
                        piblaster.setPwm(rightControlPin, motorOut)
                    }else {
                        piblaster.setPwm(leftForwardPin, 0);
                        piblaster.setPwm(leftBackwardPin, 1);
                        piblaster.setPwm(rightForwardPin, 0);
                        piblaster.setPwm(rightBackwardPin, 1);
                        piblaster.setPwm(leftControlPin, -motorOut)
                        piblaster.setPwm(rightControlPin, -motorOut)
                    }
                    console.log('Current Angle:', measuredAngle);
                    console.log('Motor Out:', motorOut);
                }
            });
        });
    }
}).start();