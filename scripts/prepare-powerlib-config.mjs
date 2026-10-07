import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';

const [baselineRoot, testRoot, outputRoot] = process.argv.slice(2);
if (!baselineRoot || !testRoot || !outputRoot) throw new Error('Usage: node prepare-powerlib.mjs <historical snapshot> <test robot> <output>');
const commit = 'a5f9d31c35f06de953dcdb5e9d7d4ce57001a65c';
const read = (relative) => fs.readFileSync(path.join(baselineRoot, relative), 'utf8');
const javaRoot = 'src/main/java/frc/robot/';
const sources = {};
const declarations = {};
for (const name of ['Feeder', 'Intake', 'Shooter', 'Spindexer', 'Turret', 'OI', 'Vision', 'Auto', 'Field', 'Sweep', 'Location', 'LED']) {
  const source = read(`${javaRoot}constants/${name}Constants.java`).replace(/\/\/[^\r\n]*/g, '');
  sources[name] = source;
  declarations[name] = Object.fromEntries([...source.matchAll(/public static final (int|double|String) (\w+)\s*=\s*([^;]+);/g)]
    .map((m) => [m[2], {type: m[1], value: m[3].trim()}]));
}
function number(name, key) {
  let value = declarations[name][key]?.value;
  if (!value) throw new Error(`Missing ${name}.${key}`);
  value = value.replace(/\b[A-Z][A-Z_0-9]*\b/g, (k) => String(number(name, k)));
  if (!/^[\d.eE+*/()\s-]+$/.test(value)) throw new Error(`Non-arithmetic value ${name}.${key}: ${value}`);
  const result = Function(`"use strict";return (${value})`)();
  assert(Number.isFinite(result));
  return result;
}
const checks = [];
const corrections = [];
function checked(destination, actual, expected, source) {
  if (typeof expected === 'number') assert(Math.abs(Number(actual) - expected) <= Math.max(1e-12, Math.abs(expected)*1e-12), `${destination}: ${actual} != ${expected}`);
  else assert.deepEqual(actual, expected, destination);
  checks.push({destination, value: expected, source});
  return expected;
}
const testGenerated = path.join(testRoot, 'power-tool/generated');
const subsystemDoc = JSON.parse(fs.readFileSync(path.join(testGenerated, 'powerlib-subsystems.json'), 'utf8'));
const constantsDoc = JSON.parse(fs.readFileSync(path.join(testGenerated, 'powerlib-constants.json'), 'utf8'));
const specs = [
  ['feeder','Feeder','FEEDER','velocity','MOTOR_CONFIGS','FEEDER_LEAD_CONFIG','FEEDER_MOTION_MAGIC_CONFIG'],
  ['intakeRoller','Intake','ROLLER','velocityTorque','ROLLER_MOTOR_CONFIGS','ROLLER_LEAD_CONFIG','ROLLER_MOTION_MAGIC_CONFIG'],
  ['intakeWrist','Intake','WRIST','absolutePosition','WRIST_MOTOR_CONFIGS','WRIST_LEAD_CONFIG','WRIST_MOTION_MAGIC_CONFIG','WRIST_CANCODER_CONFIG','INTAKE_DEFAULT'],
  ['shooter','Shooter','FLYWHEEL','velocity','FLYWHEEL_MOTOR_CONFIGS','FLYWHEEL_LEAD_CONFIG','FLYWHEEL_MOTION_MAGIC_CONFIG'],
  ['shooterHood','Shooter','HOOD','absolutePosition','HOOD_MOTOR_CONFIGS','HOOD_LEAD_CONFIG','HOOD_MOTION_MAGIC_CONFIG','HOOD_CANCODER_CONFIG','SHOOTER_HOOD_DEFAULT'],
  ['spindexer','Spindexer','SPINDEXER','velocity','SPINDEXER_MOTOR_CONFIGS','SPINDEXER_LEAD_CONFIG','SPINDEXER_MOTION_MAGIC_CONFIG'],
  ['turret','Turret','TURRET','absolutePosition','TURRET_MOTOR_CONFIGS','TURRET_LEAD_CONFIG','TURRET_MOTION_MAGIC_CONFIG','TURRET_CANCODER_CONFIG','TURRET_DEFAULT']
];
const expectedSubsystems = [];
for (const [id, owner, prefix, type, motorsKey, leadKey, mmKey, encoderKey, defaultKey] of specs) {
  const source = sources[owner];
  const list = source.match(new RegExp(`${motorsKey}\\s*=\\s*List.of\\(([\\s\\S]*?)\\);`))?.[1];
  assert(list, motorsKey);
  const motors = [...list.matchAll(/MotorConfig\.(leader|follower)\(([^)]+)\)/g)].map((m, i) => {
    const args = m[2].split(',').map((a)=>a.trim());
    const motor = {id: number(owner, args[0]), reversed: args.at(-1)==='true', neutralMode: args.find((a)=>a.startsWith('NeutralModeValue.'))?.split('.')[1] ?? 'Brake', role:m[1], motorType:'X60'};
    for (const key of ['id','reversed','neutralMode','role']) checked(`${id}.motors[${i}].${key}`, motor[key], motor[key], `${owner}Constants.${motorsKey}: ${m[0]}`);
    checks.push({destination:`${id}.motors[${i}].motorType`,value:'X60',source:'Power Tool motor-model metadata for existing TalonFX motors; legacy source does not identify X60/X44'});
    return motor;
  });
  const pid = {};
  for (const gain of ['P','I','D','G','S','V','A']) {
    const key = `${prefix}_K${gain}`;
    pid[`k${gain}`] = key in declarations[owner] ? number(owner,key) : null;
    checked(`${id}.pid.k${gain}`, pid[`k${gain}`], pid[`k${gain}`], key in declarations[owner] ? `${owner}Constants.${key}` : `${owner}Constants.${leadKey}: Optional.empty()`);
  }
  const ratios = {sensorToMechanism: declarations[owner][`${prefix}_SENSOR_TO_MECHANISM_RATIO`] ? number(owner,`${prefix}_SENSOR_TO_MECHANISM_RATIO`) : 1, rotorToSensor: declarations[owner][`${prefix}_ROTOR_TO_SENSOR_RATIO`] ? number(owner,`${prefix}_ROTOR_TO_SENSOR_RATIO`) : 1};
  for (const key of Object.keys(ratios)) checked(`${id}.ratios.${key}`,ratios[key],ratios[key],`${owner}Constants.${leadKey}`);
  const motionMagic = {cruiseVelocity: type==='absolutePosition' ? number(owner,`${prefix}_MM_CRUISE_VELOCITY`) : 0, acceleration:number(owner,`${prefix}_MM_ACCELERATION`)};
  for (const key of Object.keys(motionMagic)) checked(`${id}.motionMagic.${key}`,motionMagic[key],motionMagic[key],`${owner}Constants.${mmKey}`);
  const expected = {id,name:id,type,focEnabled:type!=='absolutePosition',torqueFF:0,motors,pid,ratios,motionMagic};
  checked(`${id}.type`,type,type, type==='absolutePosition' ? 'PositionSubsystem mapped to AbsolutePositionSubsystem' : `StateMachine.${id} declaration`);
  checked(`${id}.focEnabled`,expected.focEnabled,expected.focEnabled,type==='absolutePosition' ? 'PositionSubsystem: MotionMagicVoltage without EnableFOC (false)' : `${type==='velocityTorque'?'VelocityTorqueSubsystem':'VelocitySubsystem'}.setVelocity() (FOC enabled)`);
  checks.push({destination:`${id}.torqueFF`,value:0,source:type==='velocityTorque'?'VelocityTorqueSubsystem.setVelocity(): withFeedForward(0)':'Unused for this control type'});
  if (encoderKey) {
    const args = source.match(new RegExp(`${encoderKey}\\s*=\\s*new CancoderConfig\\(\\s*([\\s\\S]*?)\\);`))[1].split(',').map((a)=>a.trim());
    expected.cancoder = Object.fromEntries(['id','magnetOffset','discontinuityPoint'].map((key,i)=>[key,number(owner,args[i])]));
    for (const key of Object.keys(expected.cancoder)) checked(`${id}.cancoder.${key}`,expected.cancoder[key],expected.cancoder[key],`${owner}Constants.${encoderKey}`);
    expected.absolutePosition = {units:'degrees',default:number(owner,defaultKey)};
    checked(`${id}.absolutePosition.default`,expected.absolutePosition.default,expected.absolutePosition.default,`${owner}Constants.${defaultKey}; setpoints remain rotations`);
    checks.push({destination:`${id}.absolutePosition.units`,value:'degrees',source:`${owner}Constants: legacy telemetry label (no setpoint conversion)`});
  }
  const candidate = subsystemDoc.subsystems.find((s)=>s.id===id);
  if (!candidate) corrections.push({destination:id, before:null, after:'added',reason:'Missing historical hardware config'});
  else for (const [key,value] of Object.entries(expected)) {
    if (JSON.stringify(candidate[key])!==JSON.stringify(value)) corrections.push({destination:`${id}.${key}`,before:candidate[key],after:value,reason:'Match historical source; motorType remains tool metadata'});
  }
  expectedSubsystems.push(expected);
}
subsystemDoc.subsystems = expectedSubsystems;
const oldSwerve = read(`${javaRoot}subsystems/Swerve.java`);
const heading = oldSwerve.match(/HEADING_CONTROLLER = new PhoenixPIDController\(([^)]+)\)/)[1].split(',').map(Number);
const rotationRate = Number(oldSwerve.match(/MAX_DRIVE_TO_POINT_ANGULAR_RATE\s*=\s*RotationsPerSecond.of\(([^)]+)\)/)[1]) * 2*Math.PI;
for (const [section,key,value,source] of [
  ['heading','kP',heading[0],'Swerve.HEADING_CONTROLLER'],['heading','kI',heading[1],'Swerve.HEADING_CONTROLLER'],['heading','kD',heading[2],'Swerve.HEADING_CONTROLLER'],
  ['driveToPoint','maxAngularRateRadiansPerSecond',rotationRate,'Swerve.MAX_DRIVE_TO_POINT_ANGULAR_RATE (request clamp)']
]) {
  if (subsystemDoc.swerve[section][key]!==value) corrections.push({destination:`swerve.${section}.${key}`,before:subsystemDoc.swerve[section][key],after:value,reason:source});
  subsystemDoc.swerve[section][key] = checked(`swerve.${section}.${key}`,value,value,source);
}
const swerveExpected = {
 driver:{maxSpeedCoefficient:number('OI','MAX_SPEED_COEFFICIENT'),velocityScale:0.95,maxAngularRateRadiansPerSecond:1.5*2*Math.PI,joystickDeadband:0.1,skewCompensation:-0.03},
 requests:{maxAngularRateRadiansPerSecond:0.75*2*Math.PI,translationDeadbandMetersPerSecond:5.72*0.1,rotationalDeadbandRadiansPerSecond:0.75*2*Math.PI*0.1},
 driveToPoint:{maxSpeedCoefficient:0.75,maxAngularRateRadiansPerSecond:rotationRate,slowSpeedCoefficient:0.75/4,staticFrictionConstant:0.085},
 heading:{kP:heading[0],kI:heading[1],kD:heading[2]}
};
for (const [section,rows] of Object.entries(swerveExpected)) for (const [key,value] of Object.entries(rows)) {
  checked(`swerve.${section}.${key}`,subsystemDoc.swerve[section][key],value,section==='driver'?'OIConstants, SwerveDriveCommand and DriveUtil':'Swerve and DriveUtil (radians/sec conversion where applicable)');
}
function row(name,type,value,custom=true) { return {name,type,value:String(value),custom,tunable:['int','double'].includes(type)}; }
function scalarRows(owner,keys,custom=true) {
 return keys.map((key)=>{
   const declaration = declarations[owner][key];
   assert(declaration,`${owner}.${key}`);
   const value = declaration.type==='String'?declaration.value:String(number(owner,key));
   checked(`constants.${owner}.${key}`,value,value,`${owner}Constants.${key}`);
   return row(key,declaration.type,value,custom);
 });
}
constantsDoc.files['robot:OI'].constants = scalarRows('OI',Object.keys(declarations.OI)).map((r)=>({...r,custom:!['DRIVER_CONTROLLER_PORT','DEADBAND','MAX_SPEED_COEFFICIENT'].includes(r.name)}));
constantsDoc.files['robot:StateMachine'].constants = [row('DEFAULT_STATE','RobotState','RobotState.READY',false)];
constantsDoc.files['robot:Vision'].constants = [
 row('LIMELIGHT_NAME','String',declarations.Vision.LEFT_TABLE.value,false),
 row('LIMELIGHT_NAMES','String[]',`{${['LEFT_TABLE','RIGHT_TABLE','TURRET_TABLE'].map((k)=>declarations.Vision[k].value).join(', ')}}`,false),
 row('CONFIG','LimelightVisionConfig','LimelightVisionConfig.DEFAULT',false),
 ...scalarRows('Vision',Object.keys(declarations.Vision))
];
checks.push({destination:'constants.robot:Vision.CONFIG',value:'LimelightVisionConfig.DEFAULT',source:'Installed PowerLib measurement validation defaults (new library behavior)'});
const customSpecs = {
 feeder:['Feeder',['CANDI1_CAN_ID','CANDI2_CAN_ID']],
 intakeRoller:['Intake',['INTAKE_DOWN_SETPOINT','INTAKE_UP_SETPOINT','INTAKE_ROLLER_OUTPUT','ROLLER_TARGET_RPS']],
 intakeWrist:['Intake',['INTAKE_MIN','INTAKE_MAX','INTAKE_IDLE','INTAKE_FEED','INTAKE_DEFAULT']],
 shooter: ['Shooter',[]],
 shooterHood:['Shooter',['SHOOTER_HOOD_MIN','SHOOTER_HOOD_MAX','SHOOTER_HOOD_DEFAULT']],
 spindexer:['Spindexer',['LASER_1_CAN_ID','LASER_2_CAN_ID']],
 turret:['Turret',['TURRET_MIN','TURRET_MAX','TURRET_DEFAULT','TURRET_DIST_FROM_ROBOT_CENTER','TURRET_RADIUS','TURRET_LIMELIGHT_STATIC_OFFSET','TURRET_SHOOT_ANGLE_TOLERANCE_DEG','SHOOT_MAX_DRIVETRAIN_SPEED_MPS','TURRET_CAMERA_Y_OFFSET']]
};
for (const [id,[owner,keys]] of Object.entries(customSpecs)) constantsDoc.files[`subsystem:${id}`] = {constants:scalarRows(owner,keys)};
const tables = {};
for (const name of ['HOOD_ANGLES','SHOOTER_SPEEDS','FEEDER_SPEEDS']) {
 const literal = sources.Turret.match(new RegExp(`${name}\\s*=\\s*(\\{[\\s\\S]*?\\});`))[1];
 const data = JSON.parse(literal.replaceAll('{','[').replaceAll('}',']'));
 tables[name] = data;
 constantsDoc.files['subsystem:turret'].constants.push(row(name,'double[][]',literal));
 for (let i=0;i<data.length;i++) for (let j=0;j<2;j++) checked(`constants.turret.${name}[${i}][${j}]`,data[i][j],data[i][j],`TurretConstants.${name}`);
}
for (const [key,table] of [['HOOD_ANGLE_INTERPOLATOR','HOOD_ANGLES'],['SHOOTER_VELOCITY_INTERPOLATOR','SHOOTER_SPEEDS'],['FEEDER_VELOCITY_INTERPOLATOR','FEEDER_SPEEDS']]) {
 constantsDoc.files['subsystem:turret'].constants.push(row(key,'frc.powerlib.math.LinearInterpolator',`new frc.powerlib.math.LinearInterpolator(${table})`));
}
const game = JSON.parse(fs.readFileSync(path.join(testGenerated,'powerlib-game-2026.json'),'utf8'));
game.source = {repository:'FRC9410/2026-Robot',commit,cutoff:'2026-05-01T00:00:00-05:00'};
const canBus=read(`${javaRoot}constants/CanBusConstants.java`).match(/new CANBus\("([^"]+)"\)/)[1];
game.hardware={canBus,led:{canBus,...Object.fromEntries(Object.keys(declarations.LED).map((key)=>[key,number('LED',key)]))}};
checked('game.hardware.canBus',game.hardware.canBus,canBus,'CanBusConstants.CANIVORE_BUS');
for (const key of Object.keys(declarations.LED)) checked(`game.hardware.led.${key}`,game.hardware.led[key],number('LED',key),`LEDConstants.${key}`);
game.field.length=number('Field','X_MAX'); game.field.width=number('Field','Y_MAX');
delete game.field.image;
game.shots = tables.HOOD_ANGLES.map(([distance,hood])=>[distance,tables.SHOOTER_SPEEDS.find((r)=>r[0]===distance)[1],hood,tables.FEEDER_SPEEDS.find((r)=>r[0]===distance)[1]]).sort((a,b)=>a[0]-b[0]);
delete game.tolerances.hoodRotations; delete game.tolerances.autoStartMeters; delete game.tolerances.autoStartDegrees;
game.tolerances.autoTranslationMeters=number('Auto','TRANSLATION_TOLERANCE'); game.tolerances.autoRotationDegrees=number('Auto','ROTATION_TOLERANCE');
delete game.alerts;
game.field.constants=Object.fromEntries(Object.keys(declarations.Field).map((key)=>[key,number('Field',key)]));
game.field.points=Object.fromEntries([...sources.Field.matchAll(/(\w+)\s*=\s*new Translation[23]d\(([^)]+)\)/g)].map((m)=>[m[1],m[2].split(',').map(Number)]));
for (const [key,value] of Object.entries(game.field.points)) checked(`game.field.points.${key}`,value,value,`FieldConstants.${key}`);
game.location=Object.fromEntries(Object.keys(declarations.Location).map((key)=>[key,number('Location',key)]));
game.sweep={margin:number('Sweep','SWEEP_MARGIN'),driveSpeed:number('Sweep','SWEEP_DRIVE_SPEED')};
const points=game.field.points;
game.sweep.center=[(points.NEU_TOP_LEFT[0]+points.NEU_TOP_RIGHT[0])/2,(points.NEU_TOP_LEFT[1]+points.NEU_BOTTOM_LEFT[1])/2];
game.sweep.targets={top:[game.sweep.center[0],points.NEU_TOP_LEFT[1]-game.sweep.margin,90],bottom:[game.sweep.center[0],points.NEU_BOTTOM_LEFT[1]+game.sweep.margin,-90],left:[points.NEU_TOP_LEFT[0]+game.sweep.margin,game.sweep.center[1],180],right:[points.NEU_TOP_RIGHT[0]-game.sweep.margin,game.sweep.center[1],0]};
for (const [key,value] of Object.entries(game.sweep)) checked(`game.sweep.${key}`,value,value,`SweepConstants.${key}`);
const autoMatches = [...sources.Auto.matchAll(/(RED_LEFT|RED_RIGHT|BLUE_LEFT|BLUE_RIGHT)_(\d+)\s*=\s*new Pose2d\(([^,]+),\s*([^,]+),\s*Rotation2d.fromDegrees\(([^)]+)\)\)/g)];
for (const [label,prefix] of [['Red Left','RED_LEFT'],['Red Right','RED_RIGHT'],['Blue Left','BLUE_LEFT'],['Blue Right','BLUE_RIGHT']]) {
 const poses = autoMatches.filter((m)=>m[1]===prefix).sort((a,b)=>Number(a[2])-Number(b[2])).map((m)=>m.slice(3).map(Number));
 assert.equal(poses.length,7);
 checked(`game.autos.${label}`,game.autos[label],poses,`AutoConstants.${prefix}_1..7`);
}
checked('game.shots',game.shots,game.shots,'Joined TurretConstants HOOD_ANGLES, SHOOTER_SPEEDS, FEEDER_SPEEDS');
for (const [key,sourceKey] of Object.entries({deployed:'INTAKE_MAX',idle:'INTAKE_IDLE',feed:'INTAKE_FEED',stowed:'INTAKE_MIN'})) checked(`game.intake.${key}`,game.intake[key],number('Intake',sourceKey),`IntakeConstants.${sourceKey}`);
checked('game.field.blueHub',game.field.blueHub,[4.62,4],'FieldConstants.HOPPER_BLUE');
checked('game.field.redHub',game.field.redHub,[11.5,4],'FieldConstants.HOPPER_RED');
for (const [key,value] of Object.entries(game.field.constants)) checked(`game.field.constants.${key}`,value,number('Field',key),`FieldConstants.${key}`);
for (const [key,value] of Object.entries(game.location)) checked(`game.location.${key}`,value,number('Location',key),`LocationConstants.${key}`);
const container = read(`${javaRoot}RobotContainer.java`);
assert(container.includes('intakeRoller.setVelocity(145)') && container.includes('intakeRoller.setVelocity(-100)'));
checked('game.intake.collectRps',game.intake.collectRps,145,'RobotContainer.configureBindings');
checked('game.intake.ejectRps',game.intake.ejectRps,-100,'RobotContainer.configureBindings');
const state = read(`${javaRoot}subsystems/StateMachine.java`);
assert(state.includes(' - 2;') && state.includes('double tol = 5.0;'));
checked('game.tolerances.shooterRps',game.tolerances.shooterRps,2,'StateMachine.runShootingToTarget');
checked('game.tolerances.headingDegrees',game.tolerances.headingDegrees,5,'StateMachine.runShootingToTarget');
assert.equal(game.team,9410);
checked('game.cameras',game.cameras.map((c)=>c.name),['limelight-left','limelight-right','limelight-turret'],'VisionConstants.LEFT_TABLE, RIGHT_TABLE, TURRET_TABLE');
const selection = {version:1, selectedTopics:[], monitorDrawerOpen:false, sidebarExpandedSection:'subsystem'};
fs.mkdirSync(outputRoot,{recursive:true});
const write = (name,data)=>fs.writeFileSync(path.join(outputRoot,name),JSON.stringify(data,null,2)+'\n');
write('powerlib-subsystems.json',subsystemDoc);
write('powerlib-constants.json',constantsDoc);
write('powerlib-tuning-selection.json',selection);
write('powerlib-game-2026.json',game);
write('comparison.json',{baseline:{repository:'FRC9410/2026-Robot',commit,cutoff:'2026-05-01T00:00:00-05:00'},testRobot:'library-test-robot',checks,corrections,notes:[
 'Sweep is navigation software, not a motor subsystem. Turret config exists in the baseline constants but is not instantiated by its StateMachine.',
 'The game JSON is supplementary test-robot reference data. Stock Power Tool only generates subsystems, constants, and tuning-selection JSON; its Update Code action does not consume the game file.',
 'X60 is test-robot simulation metadata, not hardware model information established by legacy TalonFX declarations.',
 'Position telemetry labels remain degrees to match the source; all default positions and setpoints remain rotations.',
 'New library validation uses LimelightVisionConfig.DEFAULT. Empty tuning selections are UI state, not calibration.',
 'TunerConstants calibration is already identical to the baseline; only comments and whitespace differ.'
]});
console.log(`Prepared four JSON files; ${checks.length} reference checks passed; ${corrections.length} candidate differences corrected.`);
console.log(JSON.stringify(corrections,null,2));
