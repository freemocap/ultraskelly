import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { buildSkeleton } from '/skeleton.js';

const $ = id => document.getElementById(id);
let snapshot, busy = false;
const error = message => { $('error').textContent = message; $('error').hidden = !message; };
const deg = THREE.MathUtils.degToRad;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#111b21');
scene.fog = new THREE.Fog('#111b21', 9, 22);
const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 60);
let renderer;
try { renderer = new THREE.WebGLRenderer({antialias:true}); }
catch (e) { error('The 3D view needs WebGL2. Enable browser graphics acceleration and reload.'); throw e; }
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
$('viewport').appendChild(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true; orbit.minDistance = 2; orbit.maxDistance = 12;
function resetView() { camera.position.set(3.8, 2.6, 5.3); orbit.target.set(0, 1.35, 0.25); orbit.update(); }
resetView(); $('view-reset').onclick = resetView;
const {waist, pan, tilt, jaw, jawZone, target, targetLine} = buildSkeleton(scene);
new ResizeObserver(()=>{const box=$('viewport');camera.aspect=box.clientWidth/box.clientHeight;camera.updateProjectionMatrix();renderer.setSize(box.clientWidth,box.clientHeight);}).observe($('viewport'));

function drawCamera(s) {
  const c=$('camera'),ctx=c.getContext('2d');ctx.fillStyle='#0b1318';ctx.fillRect(0,0,640,480);
  ctx.strokeStyle='#304953';ctx.lineWidth=2;
  for(let x=0;x<=640;x+=80){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,480);ctx.stroke();}
  for(let y=0;y<=480;y+=80){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(640,y);ctx.stroke();}
  ctx.strokeStyle='#ffb269';ctx.lineWidth=3;ctx.setLineDash([7,7]);const d=s.settings.head_deadzone;ctx.strokeRect(320-d,240-d,d*2,d*2);ctx.setLineDash([]);
  ctx.beginPath();ctx.moveTo(300,240);ctx.lineTo(340,240);ctx.moveTo(320,220);ctx.lineTo(320,260);ctx.stroke();
  if(s.camera.visible){ctx.strokeStyle='#ffb269';ctx.lineWidth=4;ctx.beginPath();ctx.arc(s.camera.x,s.camera.y,14,0,Math.PI*2);ctx.stroke();}
  $('detection').textContent=s.camera.visible?'DETECTED':'NO TARGET';$('pixel').textContent=s.camera.visible?`${s.camera.x}, ${s.camera.y} px`:'outside view / hidden';
}
function drawPlot(s) {
  const c=$('plot'),ctx=c.getContext('2d'),w=c.clientWidth,h=c.clientHeight;
  c.width=Math.round(w*2);c.height=Math.round(h*2);ctx.setTransform(2,0,0,2,0,0);
  ctx.clearRect(0,0,w,h);
  const panels=[
    ['Pan / deg',0,180,[['pan_command','#8de3c2'],['pan','#e4dfce']]],
    ['Tilt / deg',0,180,[['tilt_command','#8de3c2'],['tilt','#e4dfce']]],
    ['Waist / deg',-100,100,[['waist','#e4dfce']]],
    ['Jaw / deg',0,28,[['jaw','#e4dfce']]],
    ['Waist drive',-1,1,[['waist_throttle','#8de3c2']]],
    ['Jaw drive / '+s.mouth_state.toUpperCase(),-1,0,[['mouth_throttle','#8de3c2']]],
  ];
  const end=s.state.time,start=Math.max(0,end-30),span=Math.max(5,end-start);
  const stateColors={closed:'#59666e',opening:'#ffb269',open:'#8de3c2',closing:'#b291d9'};
  panels.forEach(([title,lo,hi,series],index)=>{
    const col=index%2,row=Math.floor(index/2),pw=w/2,ph=h/3;
    const x0=col*pw+25,y0=row*ph+22,ww=pw-32,hh=ph-34;
    ctx.font='10px monospace';ctx.fillStyle='#bfced1';ctx.fillText(title,col*pw+3,row*ph+11);
    ctx.textAlign='right';ctx.fillStyle='#e4dfce';ctx.fillText(s.state[series[0][0]].toFixed(1),(col+1)*pw-7,row*ph+11);ctx.textAlign='left';
    ctx.font='8px monospace';ctx.strokeStyle='#2b3b43';ctx.lineWidth=1;
    for(const v of [lo,(lo+hi)/2,hi]){const y=y0+(hi-v)/(hi-lo)*hh;ctx.beginPath();ctx.moveTo(x0,y);ctx.lineTo(x0+ww,y);ctx.stroke();ctx.fillStyle='#81959e';ctx.fillText(String(v),col*pw+1,y+3);}
    for(const [key,color] of series){ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();s.history.forEach((point,i)=>{const x=x0+(point.time-start)/span*ww,y=y0+(hi-point[key])/(hi-lo)*hh;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}
    if(index===5){s.history.forEach(point=>{ctx.fillStyle=stateColors[point.mouth_state]||'#59666e';ctx.fillRect(x0+(point.time-start)/span*ww,y0+hh+3,Math.max(1,.05/span*ww),3);});}
  });
}
function update(s) {
  snapshot=s;const q=s.state;
  waist.rotation.y=deg(q.waist);pan.rotation.y=deg(q.pan-90);tilt.rotation.x=deg(q.tilt-90);jaw.rotation.x=deg(q.jaw);
  target.position.fromArray(s.target.position);target.visible=s.target.enabled;
  jawZone.scale.set(s.settings.head_deadzone/320,s.settings.head_deadzone/240,1);
  targetLine.visible=target.visible;targetLine.geometry.dispose();targetLine.geometry=new THREE.BufferGeometry().setFromPoints([target.position,new THREE.Vector3(target.position.x,0,target.position.z)]);
  $('time').textContent=q.time.toFixed(2);$('tick').textContent=`tick ${s.tick}`;$('run-state').textContent=s.running?'RUNNING':'PAUSED';$('play').textContent=s.running?'Ⅱ Pause':'▶ Run';
  for(const key of ['pan','tilt','waist','jaw'])$(key).textContent=q[key].toFixed(1)+'°';
  $('pan-command').textContent=q.pan_command.toFixed(1)+'°';$('tilt-command').textContent=q.tilt_command.toFixed(1)+'°';
  $('waist-drive').textContent=q.waist_throttle.toFixed(2);$('mouth-drive').textContent=q.mouth_throttle.toFixed(2);$('waist-mode').textContent=s.waist_mode;$('mouth-state').textContent=s.mouth_state;
  drawCamera(s);drawPlot(s);if(s.error)error(s.error);
}
function syncSettings(s) {
  for(const [key,value] of Object.entries(s.settings)){const el=$(key);if(!el)continue;if(el.type==='checkbox')el.checked=value;else el.value=value;}
  for(const key of ['yaw','pitch'])$(key+'-value').textContent=s.settings[key]+'°';
}
async function command(action,settings) {
  if(busy)return;busy=true;
  try {const response=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action,...(settings?{settings}:{})})});const data=await response.json();if(!response.ok)throw Error(data.error);error('');update(data);syncSettings(data);}
  catch(e){error(e.message);}finally{busy=false;}
}
$('play').onclick=()=>command(snapshot?.running?'pause':'run');$('step').onclick=()=>command('step');$('reset').onclick=()=>command('reset');
$('scenario').onchange=()=>command('configure',{scenario:$('scenario').value});$('visible').onchange=()=>command('configure',{visible:$('visible').checked});
for(const key of ['yaw','pitch']) {$(key).oninput=()=>$(key+'-value').textContent=$(key).value+'°';$(key).onchange=()=>command('configure',{scenario:'manual',[key]:Number($(key).value)});}
$('tuning').onsubmit=event=>{event.preventDefault();const values={};for(const key of ['head_speed','head_deadzone','waist_gain','sensor_hz','servo_speed','waist_speed'])values[key]=Number($(key).value);command('configure',values);};
$('export').onclick=()=>{if(!snapshot)return;const blob=new Blob([JSON.stringify({format:'ultraskelly-sim-v1',settings:snapshot.settings,history:snapshot.history},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='ultraskelly-sim.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
let first=true;
async function poll(){try{const response=await fetch('/api/state');if(!response.ok)throw Error('State request failed');const s=await response.json();if(!busy){update(s);if(first){syncSettings(s);first=false;}}$('connection').textContent='● Local server connected';}catch(e){$('connection').textContent='○ Disconnected';error('Local server disconnected. Restart python -m ultraskelly.simulator.');}finally{setTimeout(poll,100);}}
poll();renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);});
