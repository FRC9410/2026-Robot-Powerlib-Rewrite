import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';

const [robotRoot, verifiedJsonRoot] = process.argv.slice(2);
if (!robotRoot || !verifiedJsonRoot) throw new Error('Usage: node verify-powerlib-config.mjs <robot root> <verified JSON directory>');
const readJson = (root, file) => JSON.parse(fs.readFileSync(path.join(root,file),'utf8').replace(/^\uFEFF/,''));
const generated = path.join(robotRoot,'power-tool/generated');
const expected = readJson(verifiedJsonRoot,'powerlib-subsystems.json');
const actual = readJson(generated,'powerlib-subsystems.json');
function equivalent(a,b,label) {
 if (typeof a==='number' && typeof b==='string' && b.trim()!=='' && Number.isFinite(Number(b))) b=Number(b);
 if (Array.isArray(a)) { assert(Array.isArray(b),label); assert.equal(a.length,b.length,label); a.forEach((v,i)=>equivalent(v,b[i],`${label}[${i}]`)); }
 else if (a && typeof a==='object') for (const [key,value] of Object.entries(a)) equivalent(value,b[key],`${label}.${key}`);
 else if (typeof a==='number') assert(Math.abs(a-b)<=Math.max(1e-12,Math.abs(a)*1e-12),`${label}: ${a} != ${b}`);
 else assert.deepEqual(b,a,label);
}
equivalent(expected,actual,'subsystems');
for (const file of ['powerlib-constants.json','powerlib-game-2026.json','powerlib-tuning-selection.json']) equivalent(readJson(verifiedJsonRoot,file),readJson(generated,file),file);
let checked=0;
const java = (name) => fs.readFileSync(path.join(robotRoot,`src/main/java/frc/robot/constants/${name}Constants.java`),'utf8');
function value(source,key) {
 const matches=[...source.matchAll(new RegExp(`\\b${key}\\s*=\\s*([\\s\\S]*?);`,'g'))];
 assert.equal(matches.length,1,`${key}: missing or duplicated declaration`);
 return matches[0][1].trim();
}
function check(source,key,expectedValue) {
 let text=value(source,key);
 let result;
 if (text==='Optional.empty()') result=null;
 else {
  text=text.replace(/^Optional.of\((.*)\)$/s,'$1').replace(/^NeutralModeValue\./,'');
  result=typeof expectedValue==='number'?Number(text):typeof expectedValue==='boolean'?text==='true':typeof expectedValue==='string'?text.replace(/^"(.*)"$/s,'$1'):text;
 }
 equivalent(expectedValue,result,key);
 checked++;
}
const pascal=(name)=>name[0].toUpperCase()+name.slice(1);
for (const subsystem of expected.subsystems) {
 const source=java(pascal(subsystem.name));
 let followers=0;
 for (const motor of subsystem.motors) {
  const prefix=motor.role==='leader'?'LEADER':`FOLLOWER_${++followers}`;
  check(source,`${prefix}_MOTOR_ID`,motor.id);
  check(source,`${prefix}_NEUTRAL_MODE`,motor.neutralMode);
  check(source,`${prefix}_REVERSED`,motor.reversed);
 }
 for (const [key,val] of Object.entries(subsystem.pid)) check(source,key.toUpperCase(),val);
 check(source,'SENSOR_TO_MECHANISM_RATIO',subsystem.ratios.sensorToMechanism);
 check(source,'ROTOR_TO_SENSOR_RATIO',subsystem.ratios.rotorToSensor);
 check(source,'MOTION_MAGIC_CRUISE_VELOCITY',subsystem.motionMagic.cruiseVelocity);
 check(source,'MOTION_MAGIC_ACCELERATION',subsystem.motionMagic.acceleration);
 check(source,'FOC_ENABLED',subsystem.focEnabled);
 if (subsystem.cancoder) {
  check(source,'CANCODER_ID',subsystem.cancoder.id);
  check(source,'CANCODER_MAGNET_OFFSET',subsystem.cancoder.magnetOffset);
  check(source,'CANCODER_DISCONTINUITY_POINT',subsystem.cancoder.discontinuityPoint);
  check(source,'DEFAULT_POSITION',subsystem.absolutePosition.default);
  check(source,'POSITION_UNITS',subsystem.absolutePosition.units);
 } else check(source,'TORQUE_FF',subsystem.torqueFF);
}
const swerveNames={
 driver:{maxSpeedCoefficient:'DRIVER_MAX_SPEED_COEFFICIENT',velocityScale:'DRIVER_VELOCITY_SCALE',maxAngularRateRadiansPerSecond:'DRIVER_MAX_ANGULAR_RATE_RADIANS_PER_SECOND',joystickDeadband:'DRIVER_JOYSTICK_DEADBAND',skewCompensation:'DRIVER_SKEW_COMPENSATION'},
 requests:{maxAngularRateRadiansPerSecond:'REQUEST_MAX_ANGULAR_RATE_RADIANS_PER_SECOND',translationDeadbandMetersPerSecond:'REQUEST_TRANSLATION_DEADBAND_METERS_PER_SECOND',rotationalDeadbandRadiansPerSecond:'REQUEST_ROTATIONAL_DEADBAND_RADIANS_PER_SECOND'},
 driveToPoint:{maxSpeedCoefficient:'DRIVE_TO_POINT_MAX_SPEED_COEFFICIENT',maxAngularRateRadiansPerSecond:'DRIVE_TO_POINT_MAX_ANGULAR_RATE_RADIANS_PER_SECOND',slowSpeedCoefficient:'DRIVE_TO_POINT_SLOW_SPEED_COEFFICIENT',staticFrictionConstant:'DRIVE_TO_POINT_STATIC_FRICTION_CONSTANT'},
 heading:{kP:'HEADING_KP',kI:'HEADING_KI',kD:'HEADING_KD'}
};
for (const [section,rows] of Object.entries(swerveNames)) for (const [key,javaName] of Object.entries(rows)) check(java('Swerve'),javaName,expected.swerve[section][key]);
for (const [key,file] of Object.entries(readJson(verifiedJsonRoot,'powerlib-constants.json').files)) {
 const [kind,id]=key.split(':');
 const name=kind==='subsystem'?pascal(expected.subsystems.find((s)=>s.id===id).name):id;
 const source=java(name);
 for (const row of file.constants) {
  const actualValue=value(source,row.name);
  assert.equal(actualValue.replace(/\s+/g,''),row.value.replace(/\s+/g,''),`${key}.${row.name}`);
  checked++;
 }
}
const state=fs.readFileSync(path.join(robotRoot,'src/main/java/frc/robot/subsystems/StateMachine.java'),'utf8');
for (const subsystem of expected.subsystems) assert(state.includes(` ${subsystem.id} = new `),`Missing StateMachine.${subsystem.id}`);
const hardware=readJson(verifiedJsonRoot,'powerlib-game-2026.json').hardware;
for (const [key,val] of Object.entries(hardware.led)) if (key!=='canBus') check(java('LED'),key,val);
check(java('CanBus'),'CANIVORE_BUS_NAME',hardware.canBus);
assert.equal(value(java('LED'),'CANDLE_CAN_BUS'),'CanBusConstants.CANIVORE_BUS_NAME');
checked++;
const lock=readJson(path.join(robotRoot,'power-tool'),'package-lock.json');
const installed=readJson(path.join(robotRoot,'power-tool/node_modules'),'.package-lock.json');
let dependencies=0;
for (const [key,entry] of Object.entries(lock.packages)) {
 if (!key) continue;
 const disk=path.join(robotRoot,'power-tool',key,'package.json');
 if (!fs.existsSync(disk)) { assert(entry.optional,`Missing required dependency ${key}`); continue; }
 assert.equal(readJson(path.dirname(disk),'package.json').version,entry.version,`Installed version mismatch: ${key}`);
 assert.equal(installed.packages[key]?.version,entry.version,`Installed lock mismatch: ${key}`);
 dependencies++;
}
console.log(`Verified ${checked} Java configuration values, all four JSON files, seven subsystem initializations, and ${dependencies} installed package versions against the lockfile.`);
