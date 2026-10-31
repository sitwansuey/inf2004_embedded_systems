/* Display fields populated by the shared EDR16 receiver or explicit imports. */
'use strict';
const groups = [
  ['navigation','Left IR','03','Raw left sensor reading.',['ir_left']],
  ['navigation','Right IR','03','Raw right sensor reading.',['ir_right']],
  ['navigation','Barcode IR','03','Barcode sensor reading and decoded data.',['ir_barcode','barcode']],
  ['motion','Left wheel encoder','02','Packed counts wrap at 4096; speed requires calibrated wheel feedback.',['encoder_left','left_speed_mps']],
  ['motion','Right wheel encoder','02','Packed counts wrap at 4096; speed requires calibrated wheel feedback.',['encoder_right','right_speed_mps']],
  ['terrain','Accelerometer','04','Acceleration and impact information from the GY-511.',['accel_x_mps2','accel_y_mps2','accel_z_mps2','impact','imu_status']],
  ['terrain','Magnetometer','04','Heading uses the magnetometer. Pitch, roll and turn rate are derived estimates; the GY-511 has no gyroscope.',['mag_x_ut','mag_y_ut','mag_z_ut','heading_deg','pitch_deg','roll_deg','turn_rate_dps','valid']],
  ['obstacle','Ultrasonic Ranger','05','Range and obstacle geometry from the ultrasonic scan.',['distance_cm','width_cm','depth_cm','left_clearance_cm','right_clearance_cm']],
  ['terrain','Terrain & hump analysis','04','Derived motion and hump estimates.',['motion_class','hump_state','hump_height_cm','run_max_height_cm','valid']],
  ['obstacle','Servo Motor','05','Scan actuator position and avoidance decisions.',['state','servo_angle_deg','bypass_side','route']],
  ['motion','Motor control','02','Commanded PWM output is separate from measured wheel motion.',['state','direction','power_percent','fault_flags','distance_m','last_error','last_command_ms']],
  ['system','Vehicle controller','05','Mission state, safety and telemetry timing.',['state','fault_code','fault_message','safety_state','uptime_ms','telemetry_period_ms','latency_ms','sequence']]
];
// Topic names are case-sensitive. Keep the two roots exactly as agreed.
const wireTopic = 'Robopico/sensors/edr16/v1';
const topicRoots = { sensors: 'Robopico/sensors', actuators: 'Robopico/Actuators' };
const sectionTopics = {
  'Left IR': ['sensors','left_ir'],
  'Right IR': ['sensors','right_ir'],
  'Barcode IR': ['sensors','barcode_ir'],
  'Left wheel encoder': ['sensors','left_encoder'],
  'Right wheel encoder': ['sensors','right_encoder'],
  'Accelerometer': ['sensors','accelerometer'],
  'Magnetometer': ['sensors','magnetometer'],
  'Ultrasonic Ranger': ['sensors','ultrasonic_ranger'],
  'Terrain & hump analysis': ['sensors','terrain'],
  'Servo Motor': ['actuators','servo_motor'],
  'Motor control': ['actuators','motors'],
  'Vehicle controller': ['actuators','vehicle_controller']
};
const topicBindings = groups.flatMap(([key,title,,,fields])=>{
  const [category,section]=sectionTopics[title];
  return fields.map(field=>({topic:`${topicRoots[category]}/${section}/${field}`,key,field}));
});
const eventTopic = `${topicRoots.actuators}/vehicle_controller/events`;
let mqttPacket = {};
const labels = {
  ir_left:['Raw reading'],ir_barcode:['Raw reading'],ir_right:['Raw reading'],
  fault_flags:['Motor fault bits'],encoder_left:['Pulse count (mod 4096)'],encoder_right:['Pulse count (mod 4096)'],left_speed_mps:['Wheel speed','m/s'],right_speed_mps:['Wheel speed','m/s'],
  accel_x_mps2:['Acceleration X','m/s²'],accel_y_mps2:['Acceleration Y','m/s²'],accel_z_mps2:['Acceleration Z','m/s²'],
  mag_x_ut:['Magnetic field X','µT'],mag_y_ut:['Magnetic field Y','µT'],mag_z_ut:['Magnetic field Z','µT'],
  heading_deg:['Heading / yaw alias','°'],pitch_deg:['Pitch','°'],roll_deg:['Roll','°'],turn_rate_dps:['Estimated turn rate','°/s'],
  distance_cm:['Obstacle distance','cm'],width_cm:['Width','cm'],depth_cm:['Depth','cm'],left_clearance_cm:['Left clearance','cm'],right_clearance_cm:['Right clearance','cm'],
  line_error:['Line position error'],hump_height_cm:['Current hump peak','cm'],run_max_height_cm:['Run maximum height','cm'],servo_angle_deg:['Servo angle','°'],
  power_percent:['Commanded power','%'],distance_m:['Travelled distance','m'],last_command_ms:['Command timestamp','ms'],heartbeat_age_ms:['Heartbeat age','ms'],
  uptime_ms:['Robot uptime','ms'],telemetry_period_ms:['Telemetry period','ms'],latency_ms:['Reported delivery latency','ms'],valid:['Estimate validity']
};
function fieldLabel(field) { return labels[field] || [field.replaceAll('_',' ').replace(/^./,c=>c.toUpperCase())]; }
let receivedAt = null;
const el = id => document.getElementById(id);
function text(parent, tag, value, cls) { const n=document.createElement(tag); n.textContent=value; if(cls)n.className=cls; parent.append(n); return n; }
function display(v) { return v === null || v === undefined ? '—' : typeof v==='boolean' ? (v ? 'Yes' : 'No') : String(v); }
function render(packet={}, mode='Waiting') {
  if(mode!=='MQTT')mqttPacket={};
  el('summary').replaceChildren(); el('panels').replaceChildren();
  [['Mission state',packet.system?.state],['Motion',packet.motion?.state],['Terrain validity',packet.terrain?.valid],['Fault code',packet.system?.fault_code]].forEach(([label,value])=>{
    const card=text(el('summary'),'div','', 'summary-card');text(card,'span',label,'muted');text(card,'strong',display(value),'value');
  });
  groups.forEach(([key,title,owner,note,fields])=>{
    const panel=text(el('panels'),'section','','panel');panel.id='section-'+title.toLowerCase().replaceAll(/[^a-z0-9]+/g,'-');const head=text(panel,'div','','panel-heading');const titleBox=text(head,'div','');text(titleBox,'p',key.toUpperCase(),'eyebrow');text(titleBox,'h2',title);text(head,'span',`Buddy ${owner}`,'owner');
    const supplied=fields.filter(field=>packet[key]?.[field]!==undefined && packet[key]?.[field]!==null).length;
    text(panel,'p',supplied ? `${supplied}/${fields.length} readings supplied · ${mode==='Demo'?'Demo snapshot':mode==='MQTT'?'MQTT messages':'Imported snapshot'}` : 'Awaiting sensor data','sensor-status');
    const [category,section]=sectionTopics[title];
    text(panel,'p',`MQTT: ${wireTopic}`,'topic-path');
    const list=text(panel,'dl','','fields');
    fields.forEach(field=>{
      const [label,unit]=fieldLabel(field);
      const row=text(list,'div','','field');
      row.title=`Display field: ${topicRoots[category]}/${section}/${field}`;
      text(row,'dt',label);
      const dd=text(row,'dd',display(packet[key]?.[field]));
      if(unit)text(dd,'span',unit,'unit');
      text(row,'code',`Display field: ${section}/${field}`,'field-topic');
    });
    text(panel,'p',note,'panel-note');
  });
  el('events').replaceChildren();const events=Array.isArray(packet.events)?packet.events.slice(-100).reverse():[];
  if(!events.length){const row=text(el('events'),'tr','');const cell=text(row,'td','No events received.');cell.colSpan=4;}
  events.forEach(event=>{const row=text(el('events'),'tr','');['time','source','level','message'].forEach(key=>text(row,'td',display(event?.[key])));});
  el('event-count').textContent=`${events.length} events`;receivedAt=mode==='Waiting'?null:Date.now();
  el('connection').textContent=mode==='Waiting'?'Awaiting data':mode==='Demo'?'Demo data':mode==='MQTT'?'MQTT data supplied':'Imported snapshot';
  el('notice').textContent=mode==='Waiting'?'Dashboard shell · No robot connection. Values appear when telemetry is supplied.':mode==='MQTT'?'MQTT topic updates supplied · Broker connection is managed by the transport adapter.':`${mode==='Demo'?'DEMO · Synthetic values':'IMPORTED · File snapshot'} · No live robot connection.`;
  el('updated').textContent=receivedAt?`Last received: ${new Date(receivedAt).toLocaleTimeString()}`:'Last received: —';
}
function validate(packet){if(!packet||typeof packet!=='object'||Array.isArray(packet))throw new Error('Expected a JSON object.');groups.forEach(([key])=>{if(packet[key]!=null&&(typeof packet[key]!=='object'||Array.isArray(packet[key])))throw new Error(`${key} must be an object.`);});if(packet.events!=null&&(!Array.isArray(packet.events)||packet.events.some(e=>!e||typeof e!=='object'||Array.isArray(e))))throw new Error('events must be an array of objects.');return packet;}
el('demo').addEventListener('click',()=>render({motion:{state:'MOVING',direction:'FORWARD',power_percent:35,left_speed_mps:0.24,right_speed_mps:0.25,distance_m:3.8,encoder_left:420,encoder_right:424,last_error:0,last_command_ms:12000},navigation:{state:'TRACKING',ir_left:650,ir_barcode:2100,ir_right:720,line_error:0.03,barcode:'RIGHT',waypoint:'WP-02',requested_turn:'RIGHT'},terrain:{valid:true,heading_deg:92,pitch_deg:4.2,roll_deg:0.8,turn_rate_dps:1.4,accel_x_mps2:0.1,accel_y_mps2:0.02,accel_z_mps2:9.8,motion_class:'CLIMBING',impact:false,hump_state:'ASCENDING',hump_height_cm:2.1,run_max_height_cm:3.6,imu_status:'OK'},obstacle:{state:'SCANNING',distance_cm:32,width_cm:14,depth_cm:20,left_clearance_cm:40,right_clearance_cm:18,servo_angle_deg:60,bypass_side:'LEFT',route:'Pending controller'},system:{state:'LINE_TRACK',fault_code:0,fault_message:'None',safety_state:'NORMAL',uptime_ms:12500,telemetry_period_ms:10,latency_ms:24,sequence:1250},events:[{time:'00:12.4',source:'Buddy 4',level:'INFO',message:'Suspected hump; reduce speed requested.'},{time:'00:12.5',source:'Buddy 5',level:'INFO',message:'Obstacle scan started.'}]},'Demo'));
el('reset').addEventListener('click',()=>render());
el('import').addEventListener('change',async event=>{const file=event.target.files[0];if(!file)return;try{if(file.size>1048576)throw new Error('Use a file smaller than 1 MB.');render(validate(JSON.parse(await file.text())),'Imported');}catch(error){el('notice').textContent=`Import failed: ${error.message}`;}finally{event.target.value='';}});
function receiveMqtt(topic,payload) {
  const binding=topicBindings.find(item=>item.topic===topic);
  if(!binding && topic!==eventTopic)return false;
  let value=payload;
  if(typeof payload==='string') {
    try { value=JSON.parse(payload); } catch { value=payload; }
  }
  if(topic===eventTopic) {
    if(!value || typeof value!=='object' || Array.isArray(value))throw new Error('Event payload must be a JSON object.');
    mqttPacket.events=[...(mqttPacket.events || []),value].slice(-100);
  } else {
    if(value!==null && !['string','number','boolean'].includes(typeof value))throw new Error('Reading payload must be a scalar value.');
    if(typeof value==='number' && !Number.isFinite(value))throw new Error('Reading must be finite.');
    mqttPacket[binding.key] ??= {};
    mqttPacket[binding.key][binding.field]=value;
  }
  render(mqttPacket,'MQTT');
  return true;
}
window.dashboard={
  setTelemetry:packet=>render(validate(packet),'Imported'),
  receiveMqtt,
  subscriptions:[wireTopic],
  topicBindings,
  eventTopic
};
render();

// Python owns the MQTT TCP subscription; the browser polls latest readings.
if(location.protocol==='http:' || location.protocol==='https:') {
  let lastSequence=0, receiverId=null;
  async function pollBroker() {
    try {
      const response=await fetch('/api/telemetry',{cache:'no-store'});
      if(!response.ok)throw new Error('Receiver unavailable');
      const snapshot=await response.json();
      if(receiverId!==snapshot.receiver_id) {
        if(receiverId!==null)render();
        receiverId=snapshot.receiver_id;
        lastSequence=0;
      }
      for(const message of snapshot.messages) {
        if(message.sequence<=lastSequence)continue;
        try { receiveMqtt(message.topic,message.payload); }
        catch(error) { console.warn('Rejected MQTT message:',message.topic,error.message); }
      }
      lastSequence=Math.max(lastSequence,...snapshot.messages.map(message=>message.sequence));
      el('connection').textContent=snapshot.status;
      el('updated').textContent=snapshot.last_received_at
        ? `Last packet received: ${new Date(snapshot.last_received_at*1000).toLocaleTimeString()}`
        : 'Last packet received: —';
      el('notice').textContent=snapshot.status==='MQTT subscribed'
        ? `Subscribed to ${snapshot.mqtt_topic} · ${snapshot.received_packets} packets decoded · ${snapshot.rejected_packets} rejected · Latest received values; sensor freshness is not encoded.`
        : `${snapshot.status} · Displayed values may be from earlier messages.`;
    } catch(error) {
      el('connection').textContent='Dashboard receiver unavailable';
      el('notice').textContent='Start dashboard.py and open its HTTP URL to receive MQTT updates.';
    } finally { setTimeout(pollBroker,250); }
  }
  pollBroker();
}
