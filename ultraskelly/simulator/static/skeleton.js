// Schematic UltraSkelly armature shared by the simulator viewer and the live dashboard.
// Callers animate the returned joints; this module only builds geometry.
import * as THREE from 'three';

const deg = THREE.MathUtils.degToRad;

export function buildSkeleton(scene) {
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
    mesh(root,new THREE.BoxGeometry(.15,.07,.30),boneMat,side*.23,.10,.10);
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
  return {root, waist, pan, tilt, jaw, jawZone, target, targetLine, frustum, gaze};
}
