import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';

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
scene.add(new THREE.HemisphereLight(0xdcefff, 0x354738, 2.5));
const light = new THREE.DirectionalLight(0xffe8c5, 3); light.position.set(3,5,4); scene.add(light);
scene.add(new THREE.GridHelper(16, 32, 0x3e5a60, 0x23363e));
const boneMat = new THREE.MeshStandardMaterial({color:0xe4dfce, roughness:.65});
const jointMat = new THREE.MeshStandardMaterial({color:0x648c8e, metalness:.45, roughness:.35});
const darkMat = new THREE.MeshStandardMaterial({color:0x17232b, roughness:.8});
const targetMat = new THREE.MeshStandardMaterial({color:0xffb269, emissive:0xd76a23, emissiveIntensity:.4});
function mesh(parent, geometry, material, x=0,y=0,z=0) { const m=new THREE.Mesh(geometry,material); m.position.set(x,y,z); parent.add(m); return m; }
function ball(parent, x,y,z,r=.055,mat=jointMat) { return mesh(parent,new THREE.SphereGeometry(r,16,12),mat,x,y,z); }
function bone(parent,a,b,r=.035,mat=boneMat) { const p=new THREE.Vector3(...a),q=new THREE.Vector3(...b); const m=mesh(parent,new THREE.CylinderGeometry(r,r,p.distanceTo(q),12),mat); m.position.copy(p).add(q).multiplyScalar(.5); m.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),q.sub(p).normalize()); return m; }
function group(parent,x,y,z) { const g=new THREE.Group();g.position.set(x,y,z);parent.add(g);return g; }
const root = group(scene,0,0,0);
mesh(root,new THREE.CylinderGeometry(.62,.68,.08,48),darkMat,0,.04,0);
for(const side of [-1,1]) {
  const hip=[side*.17,1.05,0], knee=[side*.20,.57,.02], ankle=[side*.23,.13,0];
  bone(root,hip,knee,.052); bone(root,knee,ankle,.037); bone(root,[side*.26,.57,.02],[side*.29,.15,0],.019);
  ball(root,...knee,.065); ball(root,...hip,.075); ball(root,...ankle,.045);
  const foot=mesh(root,new THREE.BoxGeometry(.15,.07,.30),boneMat,side*.23,.10,.10);
}
const waist=group(root,0,1.05,0);
mesh(waist,new THREE.TorusGeometry(.19,.045,12,32),boneMat,0,.04,0).rotation.x=Math.PI/2;
bone(waist,[-.18,.06,0],[.18,.06,0],.06);
bone(waist,[0,.08,0],[0,.84,0],.045);
for(let i=0;i<6;i++) {
  const y=.30+i*.082, width=.22+Math.sin(i/5*Math.PI)*.08;
  const points=[]; for(let j=0;j<=32;j++){const t=j/32*Math.PI*2;points.push(new THREE.Vector3(Math.cos(t)*width,y,Math.sin(t)*.14));}
  const curve=new THREE.CatmullRomCurve3(points);mesh(waist,new THREE.TubeGeometry(curve,40,.018,6,false),boneMat);
}
bone(waist,[0,.28,.14],[0,.73,.14],.025);
for(const side of [-1,1]) {
  const shoulder=[side*.36,.78,0], elbow=[side*.48,.40,.025], wrist=[side*.52,.07,.05];
  bone(waist,[0,.76,0],shoulder,.035);ball(waist,...shoulder,.065);bone(waist,shoulder,elbow,.038);ball(waist,...elbow,.045);bone(waist,elbow,wrist,.028);ball(waist,...wrist,.035);
  for(let f=0;f<4;f++) bone(waist,[side*(.48+f*.025),.05,.05],[side*(.48+f*.025),-.12,.07],.011);
}
const neck=group(waist,0,.87,0);ball(neck,0,0,0,.075);
const pan=group(neck,0,0,0), tilt=group(pan,0,0,0);
const skull=mesh(tilt,new THREE.SphereGeometry(.19,32,24),boneMat,0,.21,.01);skull.scale.set(.9,1.13,.86);
for(const x of [-.075,.075]) {const eye=ball(tilt,x,.23,.15,.052,darkMat);eye.scale.z=.4;}
const nose=mesh(tilt,new THREE.ConeGeometry(.025,.06,3),darkMat,0,.155,.165);nose.rotation.z=Math.PI;
mesh(tilt,new THREE.BoxGeometry(.18,.055,.095),boneMat,0,.095,.12);
const jaw=group(tilt,0,.09,.015);
mesh(jaw,new THREE.BoxGeometry(.20,.045,.14),boneMat,0,-.04,.085);
bone(jaw,[-.10,0,0],[-.10,-.04,.15],.022);bone(jaw,[.10,0,0],[.10,-.04,.15],.022);
for(const x of [-.065,-.035,0,.035,.065]) mesh(tilt,new THREE.BoxGeometry(.022,.035,.025),boneMat,x,.065,.168);
const gaze = new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,.18,.2),1.3,0x8de3c2,.10,.055);tilt.add(gaze);
// Face-mounted pinhole frustum: 120 degrees horizontal, 90 vertical.
const cameraMount=group(tilt,0,.18,.2), depth=.7;
const hw=depth*Math.tan(deg(60)), hh=depth;
const corners=[[-hw,-hh,depth],[hw,-hh,depth],[hw,hh,depth],[-hw,hh,depth]];
const segments=[];
for(let i=0;i<4;i++){segments.push(new THREE.Vector3(),new THREE.Vector3(...corners[i]));segments.push(new THREE.Vector3(...corners[i]),new THREE.Vector3(...corners[(i+1)%4]));}
const frustum=new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(segments),new THREE.LineBasicMaterial({color:0x8de3c2,transparent:true,opacity:.38}));cameraMount.add(frustum);
const jawZone=new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(corners.map(v=>new THREE.Vector3(...v))),new THREE.LineBasicMaterial({color:0xffb269,transparent:true,opacity:.7}));cameraMount.add(jawZone);
const target=ball(scene,0,2,2.5,.09,targetMat);
const ring=mesh(target,new THREE.TorusGeometry(.15,.009,8,40),targetMat);ring.rotation.x=.3;
const targetLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0x735b45}));scene.add(targetLine);
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
$('tuning').onsubmit=event=>{event.preventDefault();const values={};for(const key of ['head_gain','head_deadzone','waist_gain','sensor_hz','servo_speed','waist_speed'])values[key]=Number($(key).value);command('configure',values);};
$('export').onclick=()=>{if(!snapshot)return;const blob=new Blob([JSON.stringify({format:'ultraskelly-sim-v1',settings:snapshot.settings,history:snapshot.history},null,2)],{type:'application/json'});const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='ultraskelly-sim.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
let first=true;
async function poll(){try{const response=await fetch('/api/state');if(!response.ok)throw Error('State request failed');const s=await response.json();if(!busy){update(s);if(first){syncSettings(s);first=false;}}$('connection').textContent='● Local server connected';}catch(e){$('connection').textContent='○ Disconnected';error('Local server disconnected. Restart python -m ultraskelly.simulator.');}finally{setTimeout(poll,100);}}
poll();renderer.setAnimationLoop(()=>{orbit.update();renderer.render(scene,camera);});
