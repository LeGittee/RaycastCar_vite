import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { CSM } from 'three/addons/csm/CSM.js';
import { CSMHelper } from 'three/addons/csm/CSMHelper.js';
import RAPIER from '@dimforge/rapier3d';

var spawnPosition= new RAPIER.Vector3(-77.521, 17.013, 317.74);
var spawnOrientation = new THREE.Quaternion(0.027, 0.072, 0.019, 0.997)
var dt = 1 / 60;
var camera, scene, renderer

// ---------------------------------------------------------------------------
// CSM (Cascaded Shadow Maps) - réglages modifiables
// ---------------------------------------------------------------------------
var csm;
var csmHelper = null;
var showCsmHelper = false;

// Détection tactile : sert aux boutons, au HUD et au profil d'ombres
const forceTouchControls = false; // passer à true pour tester sur desktop
const isTouchDevice = ('ontouchstart' in window) || navigator.maxTouchPoints > 0 || forceTouchControls;

const csmSettings = {
    // Profil allégé sur mobile : 2 cascades 1024² au lieu de 4 en 2048² (~8x moins de pixels d'ombre)
    cascades: isTouchDevice ? 2 : 4,
    maxFar: isTouchDevice ? 1200 : 2000,
    shadowMapSize: isTouchDevice ? 1024 : 2048,
    lightIntensity: 6,                                // 3 = un peu faible, 8 = trop blanc
    lightAngleDeg: 40,                                // inclinaison des rayons / normale au sol (10-45°)
    lightAzimuthDeg: 45,                              // orientation horizontale (0 = +X, 90 = -Z)
    lightMargin: 500,
    lightFar: 4000,
    shadowBias: -0.0005,
    fade: !isTouchDevice                              // transition douce désactivée sur mobile
};

// ---------------------------------------------------------------------------
// Inputs : clavier + manette (nomenclature XInput)
// ---------------------------------------------------------------------------
var keyState = { forward:false, backward:false, left:false, right:false, brake:false, flip:false };
var touchState = { forward:false, backward:false, left:false, right:false, flip:false };
var padState = {
    forward:false, backward:false, left:false, right:false, brake:false, flip:false,
    analog: { throttle:0, reverse:0, steer:0, cameraX:0, cameraY:0 } // RT, LT, stick gauche X, stick droit X/Y
};

// Valeurs combinées (digital + analogique) utilisées par la physique
var throttleAmount = 0; // 0..1  (RT / A / flèche haut)
var reverseAmount  = 0; // 0..1  (LT / X / flèche bas)
var steerAmount    = 0; // -1..1 (stick gauche / D-pad / flèches gauche-droite)
const analogDeadzone = 0.12;
const steerCurveExponent = 2.0; // >1 : rotation douce près du centre du stick

// Mapping standard de la Gamepad API (Xbox : A=0, B=1, X=2, Y=3, D-pad G=14, D=15)
const gamepadMapping = {
    forward:  0,  // A  : accélérer
    brake:    1,  // B  : frein à main
    backward: 2,  // X  : marche arrière
    flip:     3,  // Y  : retourner le véhicule
    left:     14, // D-pad gauche : tourner à gauche
    right:    15  // D-pad droit  : tourner à droite
};
var activeGamepadIndex = null;
const gamepadHelperButton = 8; // View / Back sur Xbox
var helperButtonWasPressed = false; // détection de front pour ne basculer qu'une fois

//sound of engine
var audio = new Audio("assets/engine_snd.mp3");
audio.loop = true;
audio.preservesPitch = true;
//audio.playbackRate = 0.5;

//offset local vector for the camera position
var offsetCam= {x:-18, y:5, z:0};
var camMoveCoef = 0.07;
var camAimCoef = 0.2;
//initialize camera positions
var camCurrentPos = AddTwoVec3(offsetCam, spawnPosition);
var targetCurrentPos = spawnPosition;

// ---------------------------------------------------------------------------
// Rotation de la caméra : stick droit + glisser souris/tactile, retour auto
// ---------------------------------------------------------------------------
var cameraYaw = 0;              // angle accumulé autour de la voiture (rad)
var cameraPitch = 0;            // décalage d'élévation (rad, borné)
var cameraIdleTime = 0;         // temps depuis la dernière commande de rotation
var dragActive = false;
var dragPointerId = null;
var lastPointerX = 0, lastPointerY = 0;
var lastFrameTime = performance.now();

const cameraSettings = {
    yawRate: 2.5,               // rad/s à plein stick (~143°/s)
    pitchRate: 1.5,             // rad/s à plein stick (~86°/s)
    mouseSensitivity: 0.008,    // rad par pixel
    pitchMin: -0.44,            // -25°
    pitchMax: 0.79,             // +45°
    returnDelay: 0.25,          // s avant de revenir derrière la voiture
    returnDuration: 0.6         // durée approximative du retour
};

// To be synced
var meshes=[], bodies=[];
var groundRawPoints, groundIndices, chassisRawPoints, chassisBBoxSize;
var wheelList=[];
let wheelPathList= ['assets/wheel.glb'];
let wheelRadiusList= [];

// parameters of the car
var chassisWidth, wheelBase;
let hoverDist= 2.1;
let wheelRadius= "not used";
let strength= 3.7;
let damping= 0.2;
let chassisMass= 1;
let maxSpeed= 500;
let maxTurnSpeed= 50;
let accGain= 0.06;
let decGain= 0.05;
let brakeGain= 0.4;
let turnGain= 26;
let maxWheelTurn= Math.PI*0.25;
let grip= 3;
let drag= 0.1;
let wheelSpincoeff= 1;

//loading manager to run rapier only when models are loaded
const ldManager = new THREE.LoadingManager()
const loader = new GLTFLoader(ldManager);

//start the physics engine when the models are loaded
ldManager.onLoad = function() { console.log( 'Loading complete!'); initRapier() }


initThree();
animate();

function initThree() {
    //create a div to show the 3d scene and insert it between top and bottom bar
    let gameDiv = document.createElement('div');
    gameDiv.classList.add('game');
    document.body.appendChild(gameDiv)

    // scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color( 0x00baff );
    scene.fog = new THREE.Fog( 0x000000, 500, 10000 );

    // camera
    camera = new THREE.PerspectiveCamera( 51.6150, window.innerWidth / (window.innerHeight) , 0.5, 10000 );
    camera.position.copy(camCurrentPos);
    //camera.quaternion.setFromAxisAngle(new THREE.Vector3(0,1,0), -Math.PI/2);
    scene.add(camera);

    // lights
    scene.add( new THREE.AmbientLight( 0x666666 ) );

    // Direction des rayons : inclinaison par rapport à la verticale + azimut au sol
    const lightAngle = THREE.MathUtils.degToRad(csmSettings.lightAngleDeg);
    const lightAzimuth = THREE.MathUtils.degToRad(csmSettings.lightAzimuthDeg);
    const lightDirection = new THREE.Vector3(
        Math.sin(lightAngle) * Math.cos(lightAzimuth),
        -Math.cos(lightAngle),
        -Math.sin(lightAngle) * Math.sin(lightAzimuth)
    ).normalize();

    // CSM : remplace l'ancienne DirectionalLight unique par 4 lumières en cascades.
    csm = new CSM({
        camera: camera,
        parent: scene,
        cascades: csmSettings.cascades,
        maxFar: csmSettings.maxFar,
        mode: 'practical',
        shadowMapSize: csmSettings.shadowMapSize,
        shadowBias: csmSettings.shadowBias,
        lightDirection: lightDirection,
        lightIntensity: csmSettings.lightIntensity,
        lightNear: 1,
        lightFar: csmSettings.lightFar,
        lightMargin: csmSettings.lightMargin
    });
    csm.fade = csmSettings.fade;
    csm.updateFrustums();

    // Touche H : affiche/masque le helper de visualisation des cascades
    document.addEventListener('keydown', function (event) {
        if (event.key === 'h' || event.key === 'H') {
            toggleCsmHelper();
        }
    });

    //create placeholder wheel models
    /*
    for (let j=0; j<wheelList.length; j++) {
        wheelList[j]= (new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.2, 6), cubeMaterial));
        wheelList[j].castShadow= true;
        wheelList[j].receiveShadow= true;
        scene.add(wheelList[j]);
    }*/

    //import the wheel models
    for (let k=0; k<4; k++){
        loader.load( wheelPathList[0], function ( gltf ) {
            wheelList[k]= gltf.scene;
            wheelList[k].castShadow= true;
            wheelList[k].receiveShadow= true;
            wheelRadiusList[k]= new THREE.Box3().setFromObject(wheelList[k]).getSize(new THREE.Vector3()).y*0.5;
            scene.add(wheelList[k])
    
        }, undefined, function ( error ) {
    
            console.error( error );
    
        } );
        
    }

    //Vehicle chassis model
    loader.load( 'assets/chassis.glb', function ( gltf ) {

        let chassisMesh= gltf.scene;
        let chassisGeo= makeMergedGeo(chassisMesh);
        chassisRawPoints= chassisGeo.attributes.position.array;
        meshes.push(chassisMesh)
        scene.add(chassisMesh)
        //get the bounding box for car parameters
        chassisBBoxSize = new THREE.Box3().setFromObject(chassisMesh).getSize(new THREE.Vector3());
        //assign them to car parameters
        chassisWidth= chassisBBoxSize.z;
        wheelBase= chassisBBoxSize.x*0.55;

    }, undefined, function ( error ) {

        console.error( error );

    } );

    //ground model
    loader.load( 'assets/ground.glb', function ( gltf ) {

        let groundMesh= gltf.scene
        let groundGeo= makeMergedGeo(groundMesh)
        groundRawPoints = groundGeo.attributes.position.array
        groundIndices = new Uint32Array(groundGeo.getIndex().array)
        scene.add( groundMesh );

        }, undefined, function ( error ) {
    
            console.log( error );
    
        }
    ); 
    
    //rocks models
    loader.load( 'assets/rocks.glb', function ( gltf ) { scene.add(gltf.scene) }, undefined, function ( error ) {

        console.error( error );

    } );


    //rendering stuff
    renderer = new THREE.WebGLRenderer( { antialias: true } );
    renderer.setSize( window.innerWidth, window.innerHeight);
    renderer.setClearColor( scene.fog.color );
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap ;

    gameDiv.appendChild( renderer.domElement );

    renderer.gammaInput = true;
    renderer.gammaOutput = true;

    window.addEventListener( 'resize', onWindowResize, false );

    initInput(); // clavier, manette, souris/tactile (une seule fois)
    updateHelperUI(); // état initial du HUD
}

// Bascule l'affichage du HUD (indication + tableau des commandes)
function updateHelperUI() {
    const hint = document.getElementById('helper-hint');
    const panel = document.getElementById('helper-panel');
    if (hint) hint.hidden = showCsmHelper;
    if (panel) panel.hidden = !showCsmHelper;
}

// Affiche/masque le helper de debug des cascades (créé à la demande)
function toggleCsmHelper() {
    if (!csm) return;

    showCsmHelper = !showCsmHelper;

    if (showCsmHelper) {
        if (!csmHelper) {
            csmHelper = new CSMHelper(csm);
            csmHelper.visible = false;
            scene.add(csmHelper);
        }
        csmHelper.visible = true;
    } else if (csmHelper) {
        csmHelper.visible = false;
    }

    updateHelperUI();
}

// Met à jour les angles de rotation de la caméra (stick droit en vitesse + retour auto)
function updateCameraInput(dt) {
    const cx = padState.analog.cameraX;
    const cy = padState.analog.cameraY;

    if (cx !== 0 || cy !== 0) {
        cameraYaw   += cx * cameraSettings.yawRate * dt;
        // axe Y manette : pousser vers le haut élève la caméra (axe 3 = -1 en haut)
        cameraPitch -= cy * cameraSettings.pitchRate * dt;
        cameraPitch = Math.max(cameraSettings.pitchMin, Math.min(cameraSettings.pitchMax, cameraPitch));
        cameraIdleTime = 0;
    } else if (!dragActive) {
        cameraIdleTime += dt;
        if (cameraIdleTime >= cameraSettings.returnDelay) {
            // ramène l'angle dans [-PI, PI] : même orientation visuelle, retour par le plus court chemin
            cameraYaw = Math.atan2(Math.sin(cameraYaw), Math.cos(cameraYaw));
            const keep = Math.exp(-dt / (cameraSettings.returnDuration / 3));
            cameraYaw *= keep;
            cameraPitch *= keep;
        }
    }
}

// Calcule le décalage de la caméra dans le repère de la voiture
function getCameraLocalOffset() {
    const base = new THREE.Vector3(offsetCam.x, offsetCam.y, offsetCam.z);
    const spherical = new THREE.Spherical().setFromVector3(base);
    spherical.theta -= cameraYaw;
    // phi = angle polaire depuis +Y : le pitch l'élève ; on borne pour rester au-dessus du sol
    spherical.phi = Math.max(0.15, Math.min(Math.PI / 2 - 0.05, spherical.phi - cameraPitch));
    const rotated = new THREE.Vector3().setFromSpherical(spherical);
    return rotateVectorByQuaternion(rotated, chassisRB.rotation());
}

function updateSpringarmCam(){

    //make the position of the camera
    let camFinalPosition = AddTwoVec3(getCameraLocalOffset(), new THREE.Vector3().copy(chassisRB.translation()));
    let camNextPosition= AddTwoVec3(vector3ByScalar(SubtTwoVec3(camFinalPosition, camCurrentPos), camMoveCoef), camCurrentPos);
    camera.position.copy(new THREE.Vector3().copy(camNextPosition));
    //make the position of the aim target of the camera
    let targetFinalPosition = new THREE.Vector3().copy(chassisRB.translation());
    let targetNextPosition = AddTwoVec3(vector3ByScalar(SubtTwoVec3(targetFinalPosition, targetCurrentPos), camAimCoef), targetCurrentPos);
    camera.lookAt(new THREE.Vector3().copy(targetNextPosition));
    //store calculated positions for the next frame
    camCurrentPos = camera.position;
    targetCurrentPos= targetNextPosition;
}

function makeMergedGeo(loadedGeo) {
    // Merge geometries from model meshes (code copied from doppl3r KCC example - Trimesh Class)
    var geometry;
    var geometries = [];
    loadedGeo.traverse(function(child) {
      if (child.isMesh) {
        // Translate geometry from mesh origin
        geometry = child.geometry;
        geometry.rotateX(child.rotation.x);
        geometry.rotateY(child.rotation.y);
        geometry.rotateZ(child.rotation.z);
        geometry.scale(child.scale.x, child.scale.y, child.scale.z);
        geometry.translate(child.position.x, child.position.y, child.position.z);
        // Push geometry to array for merge
        geometries.push(geometry);
      }
    });
    geometry = mergeGeometries(geometries);
    return geometry;
}


function onWindowResize() {
    camera.aspect = window.innerWidth / (window.innerHeight) ;
    camera.updateProjectionMatrix();
    renderer.setSize( window.innerWidth, window.innerHeight);
    // les frustums des cascades dépendent de la projection de la caméra
    if (csm) csm.updateFrustums();
}

function animate() {

    //met à jour la matrice de la caméra avant de placer les lumières CSM
    camera.updateMatrixWorld();
    if (csm) {
        csm.update();
        if (csmHelper && csmHelper.visible) csmHelper.update();
    }

    //renders the Three js scene to the canvas
    renderer.render(scene, camera);
    //asks the browser to update the canvas ?
    requestAnimationFrame( animate );
}



//two functions (copied from chatGPT) used to get my directional vector3 using the orientation quaternion
function multiplyQuaternions(q, r) {
    return {
        w: q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z,
        x: q.w * r.x + q.x * r.w + q.y * r.z - q.z * r.y,
        y: q.w * r.y - q.x * r.z + q.y * r.w + q.z * r.x,
        z: q.w * r.z + q.x * r.y - q.y * r.x + q.z * r.w
    };
}

function rotateVectorByQuaternion(v, q) {
    // Convertir le vecteur en quaternion avec une partie w = 0
    let p = { w: 0, x: v.x, y: v.y, z: v.z };

    // Calculer q * p * q^-1 (où q^-1 est l'inverse du quaternion q)
    let q_conjugate = { w: q.w, x: -q.x, y: -q.y, z: -q.z };

    let q_p = multiplyQuaternions(q, p);
    let q_p_q_inverse = multiplyQuaternions(q_p, q_conjugate);

    // Le vecteur résultant est contenu dans la partie x, y, z du résultat
    return { x: q_p_q_inverse.x, y: q_p_q_inverse.y, z: q_p_q_inverse.z };
}

function getRayLength(rayDir, timeOfImpact){
    //give the correct value to the vector by multiplying it by ToI
    let tempVector= {
    x:rayDir.x*timeOfImpact,
    y:rayDir.y*timeOfImpact,
    z:rayDir.z*timeOfImpact
    };
    //calculate the magnitude of the vector
    return Math.sqrt(Math.pow(tempVector.x, 2)+Math.pow(tempVector.y, 2)+Math.pow(tempVector.z, 2))
}

function getVector3Magnitude(vector){
    //calculate the magnitude of the vector
    return Math.sqrt(Math.pow(vector.x, 2)+Math.pow(vector.y, 2)+Math.pow(vector.z, 2))
}

function vector3ByScalar(vec3, scalar){
    let tempVector2= {
        x:vec3.x*scalar,
        y:vec3.y*scalar,
        z:vec3.z*scalar
    };
    return {x:tempVector2.x, y:tempVector2.y, z:tempVector2.z}
}
function AddTwoVec3(vector1, vector2){
    let tempVector3= {
        x:vector1.x+vector2.x,
        y:vector1.y+vector2.y,
        z:vector1.z+vector2.z
    }
    return {x:tempVector3.x, y:tempVector3.y, z:tempVector3.z}
}
function SubtTwoVec3(vector1, vector2){
    let tempVector4= {
        x:vector1.x-vector2.x,
        y:vector1.y-vector2.y,
        z:vector1.z-vector2.z
    }
    return {x:tempVector4.x, y:tempVector4.y, z:tempVector4.z}
}
function crossProduct(vector1, vector2){
    if (Object.keys(vector1).length !== 3 || Object.keys(vector2).length !== 3) {
        throw new Error("Les vecteurs doivent avoir 3 composantes chacun.");
    }
    return {x:(vector1.y*vector2.z-vector1.z*vector2.y), y:(vector1.z*vector2.x-vector1.x*vector2.z), z:(vector1.x*vector2.y-vector1.y*vector2.x)}
}
function dotProduct(vector1, vector2) {
    // Ensure the vectors contain x, y, and z components
    if (!("x" in vector1 && "y" in vector1 && "z" in vector1) ||
        !("x" in vector2 && "y" in vector2 && "z" in vector2)) {
        throw new Error("Vectors must have x, y, and z properties.");
    }
    // Calculate the dot product
    return vector1.x * vector2.x +
           vector1.y * vector2.y +
           vector1.z * vector2.z;
}

// ---------------------------------------------------------------------------
// Gestion des entrées (une seule initialisation, puis lecture à chaque frame)
// ---------------------------------------------------------------------------
function setKeyState(event, isDown) {
    switch (event.key) {
        case "ArrowRight": keyState.right    = isDown; break;
        case "ArrowLeft":  keyState.left     = isDown; break;
        case "ArrowUp":    keyState.forward  = isDown; break;
        case "ArrowDown":  keyState.backward = isDown; break;
        case "f": case "F": keyState.flip     = isDown; break;
        case " ":          keyState.brake    = isDown; break;
        default: return;
    }
    event.preventDefault(); // empêche le défilement de la page avec les flèches / espace
}

function initInput() {
    document.addEventListener("keydown", e => setKeyState(e, true));
    document.addEventListener("keyup",   e => setKeyState(e, false));

    window.addEventListener("gamepadconnected", e => {
        activeGamepadIndex = e.gamepad.index;
        console.log("Manette connectée :", e.gamepad.id);
    });
    window.addEventListener("gamepaddisconnected", e => {
        if (activeGamepadIndex === e.gamepad.index) activeGamepadIndex = null;
        console.log("Manette déconnectée :", e.gamepad.id);
    });

    // Glisser souris / tactile : alimente les mêmes angles que le stick droit
    const canvas = renderer.domElement;
    canvas.addEventListener('pointerdown', e => {
        dragActive = true;
        dragPointerId = e.pointerId;
        lastPointerX = e.clientX;
        lastPointerY = e.clientY;
        canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener('pointermove', e => {
        if (!dragActive || e.pointerId !== dragPointerId) return;
        cameraYaw   += (e.clientX - lastPointerX) * cameraSettings.mouseSensitivity;
        cameraPitch += (e.clientY - lastPointerY) * cameraSettings.mouseSensitivity;
        cameraPitch = Math.max(cameraSettings.pitchMin, Math.min(cameraSettings.pitchMax, cameraPitch));
        lastPointerX = e.clientX;
        lastPointerY = e.clientY;
        cameraIdleTime = 0;
    });
    const endDrag = e => {
        if (e.pointerId !== dragPointerId) return;
        dragActive = false;
        dragPointerId = null;
        if (canvas.hasPointerCapture(e.pointerId)) canvas.releasePointerCapture(e.pointerId);
        cameraIdleTime = 0;
    };
    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', endDrag);

    initTouchControls();
}

// Boutons tactiles + bouton plein écran
function initTouchControls() {
    if (isTouchDevice) document.body.classList.add('touch-controls-on');

    // Un appui = une action maintenue ; on capture le pointeur pour ne pas "coller" le bouton
    document.querySelectorAll('.touch-btn[data-action]').forEach(btn => {
        const action = btn.dataset.action;

        const press = e => {
            e.preventDefault();
            touchState[action] = true;
            btn.classList.add('is-pressed');
            btn.setPointerCapture(e.pointerId);
        };
        const release = e => {
            e.preventDefault();
            touchState[action] = false;
            btn.classList.remove('is-pressed');
            if (btn.hasPointerCapture(e.pointerId)) btn.releasePointerCapture(e.pointerId);
        };

        btn.addEventListener('pointerdown', press);
        btn.addEventListener('pointerup', release);
        btn.addEventListener('pointercancel', release);
        btn.addEventListener('lostpointercapture', release);
    });

    // Plein écran : un seul bouton qui change d'icône selon l'état
    const fsBtn = document.getElementById('fullscreen-btn');
    const syncFullscreenButton = () => {
        const active = !!(document.fullscreenElement || document.webkitFullscreenElement);
        fsBtn.textContent = active ? '✕' : '⛶';
        const label = active ? 'Exit fullscreen' : 'Enter fullscreen';
        fsBtn.setAttribute('aria-label', label);
        fsBtn.title = label;
    };

    fsBtn.addEventListener('click', () => {
        const active = !!(document.fullscreenElement || document.webkitFullscreenElement);
        if (!active) {
            if (document.documentElement.requestFullscreen) document.documentElement.requestFullscreen();
            else if (document.documentElement.webkitRequestFullscreen) document.documentElement.webkitRequestFullscreen();
        } else {
            if (document.exitFullscreen) document.exitFullscreen();
            else if (document.webkitExitFullscreen) document.webkitExitFullscreen();
        }
    });

    document.addEventListener('fullscreenchange', syncFullscreenButton);
    document.addEventListener('webkitfullscreenchange', syncFullscreenButton);
    syncFullscreenButton();

    // Le HUD lui-même sert de bouton : un tap ouvre l'aide, un autre tap la ferme
    const hudHint = document.getElementById('helper-hint');
    const hudPanel = document.getElementById('helper-panel');
    [hudHint, hudPanel].forEach(el => {
        if (!el) return;
        el.setAttribute('role', 'button');
        el.setAttribute('tabindex', '0');
        el.addEventListener('pointerdown', e => {
            e.preventDefault();
            e.stopPropagation();
            toggleCsmHelper();
        });
        el.addEventListener('keydown', e => {
            if (e.key === 'Enter' || e.key === ' ') {
                e.preventDefault();
                e.stopPropagation();
                toggleCsmHelper();
            }
        });
    });
    if (isTouchDevice && hudHint) hudHint.textContent = 'Tap to open helper';
}

function pollGamepad() {
    if (!navigator.getGamepads) return;

    const pads = navigator.getGamepads();
    let pad = null;

    if (activeGamepadIndex !== null && pads[activeGamepadIndex]) {
        pad = pads[activeGamepadIndex];
    } else {
        // sinon on prend la première manette disponible
        for (let i = 0; i < pads.length; i++) {
            if (pads[i]) { pad = pads[i]; activeGamepadIndex = i; break; }
        }
    }

    if (!pad) {
        // pas de manette : on relâche tout pour éviter une touche restée "bloquée"
        padState.forward = padState.backward = padState.left = padState.right = false;
        padState.brake = padState.flip = false;
        padState.analog.throttle = padState.analog.reverse = padState.analog.steer = 0;
        padState.analog.cameraX = padState.analog.cameraY = 0;
        helperButtonWasPressed = false;
        return;
    }

    // Boutons digitaux (A, B, X, Y, D-pad)
    for (const action in gamepadMapping) {
        const button = pad.buttons[gamepadMapping[action]];
        padState[action] = !!button && (button.pressed || button.value > 0.5);
    }

    // Gâchettes : boutons 6 (LT) et 7 (RT), valeur 0..1 proportionnelle
    const rt = pad.buttons[7] ? pad.buttons[7].value : 0;
    const lt = pad.buttons[6] ? pad.buttons[6].value : 0;
    padState.analog.throttle = rt;
    padState.analog.reverse  = lt;

    // Stick gauche, axe X : -1 = gauche, +1 = droite, avec zone morte et courbe douce
    let sx = pad.axes[0] || 0;
    if (Math.abs(sx) < analogDeadzone) {
        sx = 0;
    } else {
        // remise à l'échelle [deadzone..1] -> [0..1], puis courbe puissance pour adoucir
        // les faibles inclinaisons (1 = linéaire, 2 = doux, 3 = très doux)
        const normalized = (Math.abs(sx) - analogDeadzone) / (1 - analogDeadzone);
        sx = Math.sign(sx) * Math.pow(normalized, steerCurveExponent);
    }
    padState.analog.steer = sx;

    // Stick droit : axe 2 = yaw, axe 3 = pitch (vitesse, donc zone morte simple)
    let cx = pad.axes[2] || 0;
    let cy = pad.axes[3] || 0;
    if (Math.abs(cx) < analogDeadzone) cx = 0;
    else cx = Math.sign(cx) * (Math.abs(cx) - analogDeadzone) / (1 - analogDeadzone);
    if (Math.abs(cy) < analogDeadzone) cy = 0;
    else cy = Math.sign(cy) * (Math.abs(cy) - analogDeadzone) / (1 - analogDeadzone);
    padState.analog.cameraX = cx;
    padState.analog.cameraY = cy;

    // Bouton View / Back : bascule du helper uniquement au moment de l'appui
    const helperPressed = !!pad.buttons[gamepadHelperButton] && pad.buttons[gamepadHelperButton].pressed;
    if (helperPressed && !helperButtonWasPressed) {
        toggleCsmHelper();
    }
    helperButtonWasPressed = helperPressed;
}

// Combine clavier + manette dans les variables historiques utilisées par la physique
function updateDrivingInput() {
    pollGamepad();

    const fwd   = keyState.forward  || padState.forward  || touchState.forward;
    const back  = keyState.backward || padState.backward || touchState.backward;
    const left  = keyState.left     || padState.left     || touchState.left;
    const right = keyState.right    || padState.right    || touchState.right;

    // Les touches digitales restent à pleine puissance, l'analogique module l'intensité
    throttleAmount = Math.max(fwd ? 1 : 0, padState.analog.throttle);
    reverseAmount  = Math.max(back ? 1 : 0, padState.analog.reverse);

    const digitalSteer = (right ? 1 : 0) - (left ? 1 : 0);
    steerAmount = Math.max(-1, Math.min(1, digitalSteer + padState.analog.steer));

    GoingForward  = throttleAmount > 0.01;
    GoingBackward = reverseAmount > 0.01;
    // la zone morte est déjà appliquée sur l'axe du stick dans pollGamepad()
    GoingLeft     = steerAmount < -0.01;
    GoingRight    = steerAmount > 0.01;
    Braking       = keyState.brake || padState.brake;
    Flipping      = keyState.flip || padState.flip || touchState.flip;
}

//declare variables of initRapier
var world;
var CubeBodyDesc, CubeBody, CubeColDesc, CubeCol;
var chassisBodyDesc, chassisBody, chassisColDesc, chassisCol;
var groundBodyDesc, groundBody, groundColDesc, groundCol;
var GoingForward, GoingBackward, GoingLeft, GoingRight, Braking, Flipping, WheelTurn, WheelSpin;
var chassisRB, isHoveringList, hoverDistList, com, ray;
var localUp, localDown, localFwd, localRight, posA, posB, posC, posD, wheelPointList;

function initRapier() {
    //set all the models to cast shadows + connect their materials to CSM
    scene.traverse( function ( node ) {
        if ( !node.isMesh ) return;

        node.castShadow = true;
        node.receiveShadow = true;

        const materials = Array.isArray(node.material) ? node.material : [node.material];
        materials.forEach( function ( material ) {
            if ( !material ) return;

            if ( 'metalness' in material ) {
                material.metalness = 0;
                material.shadowSide = THREE.FrontSide;
            }

            // les matériaux GLTF (Standard/Physical) doivent être branchés sur CSM
            if ( csm && material.isMeshStandardMaterial && !material.userData.csmSetup ) {
                csm.setupMaterial( material );
                material.userData.csmSetup = true;
                material.needsUpdate = true;
            }
        } );
    } );

    world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
    world.timestep = dt


    //create vehicle chassis physics
    chassisBodyDesc= RAPIER.RigidBodyDesc.dynamic();
    chassisBody= world.createRigidBody(chassisBodyDesc);
    chassisColDesc= RAPIER.ColliderDesc.convexHull(chassisRawPoints);
    chassisCol= world.createCollider(chassisColDesc, chassisBody);
    chassisBody.setTranslation( spawnPosition )
    chassisBody.setRotation( spawnOrientation )
    bodies.push(chassisBody) 

    //create the rocks physics
    var sceneObjects= scene.getObjectsByProperty("isObject3D", true);
    var rockBodies= [];
    for (let i=0; i!==sceneObjects.length; i++){
        if (sceneObjects[i].name.includes("rocks")){
            meshes.push(scene.getObjectByName(sceneObjects[i].name));
            rockBodies[i]= world.createRigidBody(RAPIER.RigidBodyDesc.dynamic());
            world.createCollider(RAPIER.ColliderDesc.convexHull(sceneObjects[i].geometry.attributes.position.array), rockBodies[i]);
            rockBodies[i].setTranslation(sceneObjects[i].position);
            bodies.push(rockBodies[i])
        };
    };

    console.log("bodies", bodies)
    console.log("meshes", meshes)


    //create ground physics
    groundBodyDesc = RAPIER.RigidBodyDesc.fixed()
    groundBody = world.createRigidBody(groundBodyDesc)
    groundColDesc = RAPIER.ColliderDesc.trimesh(groundRawPoints, groundIndices, 'FIX_INTERNAL_EDGES');
    groundCol = world.createCollider(groundColDesc, groundBody)

    //initialize drive
    GoingForward  = false;
    GoingBackward = false;
    GoingLeft     = false;
    GoingRight    = false;
    Braking       = false;
    Flipping      = false;
    WheelTurn     = 0.0;
    WheelSpin     = 0.0;
    
    //initialize hover
    chassisRB = chassisBody;
    isHoveringList = [false, false, false, false];
    hoverDistList = [0.0, 0.0, 0.0, 0.0];
    com= chassisRB.localCom(); //chassis get the center of mass
    ray = new RAPIER.Ray();
    //calculate the local position of each wheel point where we will cast the ray from
    localUp = {x:0, y:1, z:0};
    localDown = {x:0, y:-1, z:0};
    localFwd={x:1, y:0, z:0};
    localRight={x:0, y:0, z:1};
    posA= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5*-1), vector3ByScalar(localFwd, wheelBase*0.5)));
    posB= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5), vector3ByScalar(localFwd, wheelBase*0.5)));
    posC= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5*-1), vector3ByScalar(localFwd, wheelBase*0.5*-1)));
    posD= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5), vector3ByScalar(localFwd, wheelBase*0.5*-1)));
    //creer un array de positions locales
    wheelPointList = [posA, posB, posC, posD];
    
    //start the physics loop
    gameLoop();
}

//animate the models
let gameLoop = ()=> {
    world.step();

    //updates display of threejs meshes
    for(var i=0; i !== meshes.length; i++){
        meshes[i].position.copy(bodies[i].translation());
        meshes[i].quaternion.copy(bodies[i].rotation());
    }
    
    //Temps réel pour les vitesses de rotation (indépendant du framerate)
    const nowFrame = performance.now();
    let frameDt = (nowFrame - lastFrameTime) / 1000;
    lastFrameTime = nowFrame;
    frameDt = Math.min(Math.max(frameDt, 0), 0.05);

    //INPUTS : clavier + manette + souris/tactile (listeners enregistrés une seule fois)
    updateDrivingInput();
    updateCameraInput(frameDt);
    updateSpringarmCam();

    //UPDATE DRIVE
    //prevents the chassis from being put to sleep
    chassisRB.wakeUp()
    //flipping feature in case the car is flipped
    let chasRot =  chassisRB.rotation();
    let worldUp= rotateVectorByQuaternion(localUp, chasRot)
    let worldRight= rotateVectorByQuaternion(localRight, chasRot);
    let worldFwd= rotateVectorByQuaternion(localFwd, chasRot);
    if (Flipping) {
        let cosine= dotProduct(worldUp, localUp);
        if ( cosine < 0.5) {
            chassisRB.setLinvel({x:0, y:0, z:0})
            chassisRB.setAngvel({x:0, y:0, z:0})
            let newPos= AddTwoVec3(chassisRB.translation(), vector3ByScalar(localUp, hoverDist*5));
            chassisRB.setTranslation(newPos)
            //reinitialiser l orientation
            chassisRB.setRotation({x:0, y:0, z:0, w:1})
        }
    }
    //power coefficient based on the nbr of wheels touching the ground
    let powerCoeff= (isHoveringList[0]+isHoveringList[1]+isHoveringList[2]+isHoveringList[3])*0.25
    //console.log("pow coeff", powerCoeff)
    
    //commonly used physical information
    let currentVel= chassisRB.linvel();
    let currentSpeed= dotProduct(currentVel, worldFwd);
    let propSpeed= Math.min(Math.abs(currentSpeed)/maxSpeed, 1);
    let angularVel= chassisRB.angvel();
    //If going forward we want to reach our Max Speed (impulsion modulée par RT / touche digitale)
    if (GoingForward){
        let diff= maxSpeed - currentSpeed;
        //Apply an impulse proportial to the difference between our desired speed and our actual speed
        let imp= vector3ByScalar(worldFwd, (diff * accGain * chassisMass) * powerCoeff * throttleAmount);
        chassisRB.applyImpulse(imp)
    }
    //If reversing we reach -Max Speed (impulsion modulée par LT / touche digitale)
    if (GoingBackward){
        let diff= -maxSpeed - currentSpeed;
        let imp= vector3ByScalar(worldFwd, (diff * decGain * chassisMass) * powerCoeff * reverseAmount);
        chassisRB.applyImpulse(imp)
    }
    //If braking we reach 0 (frein à main : reste digital)
    if (Braking){
        let imp= vector3ByScalar(worldFwd, (-currentSpeed * brakeGain * chassisMass) * powerCoeff);
        chassisRB.applyImpulse(imp)
    }
    //Check if we can turn. We check if the magnitude of our angular velocity is less than our max turn speed. Max Turning speed depends on velocity
    let mTS= Math.min(Math.abs(currentSpeed), (1-propSpeed) * maxTurnSpeed)
    //console.log("mTS", mTS)
    //console.log("current speed", currentSpeed)
    let canTurn= (Math.abs(dotProduct(angularVel, worldUp)) < mTS)
    // Le couple de direction est proportionnel à l'enfoncement du stick gauche
    let steerPower = Math.abs(steerAmount) * powerCoeff;
    //If we can turn and we're turning left
    if (canTurn && GoingLeft){
        //Apply an angular impulse to turn us left
        let imp= vector3ByScalar(worldUp, (turnGain * chassisMass) * steerPower);
        if(GoingBackward){ imp= vector3ByScalar(imp, -1)}
        chassisRB.applyTorqueImpulse(imp)
    }
    if (canTurn && GoingRight){
        //Apply an angular impulse to turn us right
        let imp= vector3ByScalar(worldUp, (-turnGain * chassisMass) * steerPower);
        if(GoingBackward){ imp= vector3ByScalar(imp, -1)}
        chassisRB.applyTorqueImpulse(imp)
    }
        //Work out how fast we're sliding left/right and compensate according to the grip property. We just allow the car to slide if its braking
        let slideSpeed= dotProduct(worldRight, currentVel);
        if (!Braking){
        let imp= vector3ByScalar(worldRight, (-slideSpeed*chassisMass*grip)*powerCoeff);
        //Apply an impulse to compensate for sliding
        chassisRB.applyImpulse(imp)
        }
        //Apply an angular impulse to compensate for spinning
        if(!canTurn || (!GoingLeft && !GoingRight)){
        let imp= vector3ByScalar(worldUp, dotProduct(angularVel, worldUp)*(-turnGain*chassisMass)*powerCoeff);
        chassisRB.applyTorqueImpulse(imp)
        }
        //Apply Drag proportional to speed
        let imp= vector3ByScalar(currentVel, -1*(drag*chassisMass));
        chassisRB.applyImpulse(imp)


    //UPDATE HOVER 
    //Find the downward direction of the chassis in world space
    let worldDownDir = rotateVectorByQuaternion(localDown, chasRot);
    let linVel= chassisRB.linvel();
    //console.log("linVel", linVel)
    let angVel= chassisRB.angvel();
    //console.log("angVel", angVel)
    //console.log("Chassis Down Dir: ", worldDownDir);

    //for each wheel do
    for (let i=0; i<wheelPointList.length; i++) {
        isHoveringList[i]=false;
        //trouver les world coord des wheel points
        //appliquer la rotation
        let tempVar= rotateVectorByQuaternion(wheelPointList[i], chasRot)
        //appliquer la translation
        let worldWheelPoint = AddTwoVec3(tempVar, chassisRB.translation());
        //raycaster
        ray.dir= worldDownDir;
        ray.origin= worldWheelPoint;
        let hit = world.castRay(ray, 200, false, null, null, null, chassisRB);
        //console.log("wheel", i, "distance", hit.timeOfImpact);
        if (hit !== null) {
            if (hit.timeOfImpact<hoverDist) {
            let hitPoint = ray.pointAt(hit.timeOfImpact);
            //calculate point velocity at wheelpoint
            let pointVelocity = AddTwoVec3(linVel, crossProduct(angVel, SubtTwoVec3(worldWheelPoint, chassisRB.translation())));
            let diff= SubtTwoVec3(hitPoint, worldWheelPoint);
            //force of the spring 
            let f= -strength * (hoverDist - hit.timeOfImpact);
            //force of the damper 
            f= f -damping * dotProduct(pointVelocity, diff)
            let force= vector3ByScalar(vector3ByScalar(diff, f), chassisMass);
            //console.log("force at wheel", i, force)

            //Apply the force to the wheelPoint
            chassisRB.applyImpulseAtPoint(force, worldWheelPoint)
            //set the value to say we are hovering
            isHoveringList[i]= true;
            hoverDistList[i]=hit.timeOfImpact;
                }
            }
    }
    //UPDATE WHEEL MODELS
    let forwardVel= currentSpeed;
    //Orientation visuelle des roues proportionnelle au steering (stick ou touches)
    let wheelTarget = steerAmount * 0.9;
    WheelTurn += (wheelTarget - WheelTurn) * 0.2;
    if (Math.abs(WheelTurn) < 0.001) WheelTurn = 0;
    // Cap wheelspin
    WheelSpin= WheelSpin - (forwardVel*wheelSpincoeff);
    if(WheelSpin>360){
        WheelSpin= WheelSpin-360;
    } else if(WheelSpin<-360){
        WheelSpin= WheelSpin+360;
    }
    let WheelSpinRad= (WheelSpin*2*Math.PI)/360;
    //determine wheel positions
    for (let i=0; i<isHoveringList.length; i++) {
        if(isHoveringList[i]){ //when touching the ground
            //trouver les world coord des wheel points
            let worldWheelOffset= rotateVectorByQuaternion(wheelPointList[i], chasRot);
            let worldWheelPoint = AddTwoVec3(worldWheelOffset, chassisRB.translation());
            //trouver la coord du centre de la roue et appliquer la position au modele
            let wheelPos= SubtTwoVec3(AddTwoVec3(worldWheelPoint, vector3ByScalar(worldDownDir, hoverDistList[i])) , vector3ByScalar(worldDownDir, wheelRadiusList[i]));
            wheelList[i].position.copy(wheelPos);
        } else { //when not touching ground
            let worldWheelOffset= rotateVectorByQuaternion(wheelPointList[i], chasRot);
            let worldWheelPoint = AddTwoVec3(worldWheelOffset, chassisRB.translation());
            let wheelPos= SubtTwoVec3(AddTwoVec3(worldWheelPoint, vector3ByScalar(worldDownDir, hoverDist)) , vector3ByScalar(worldDownDir, wheelRadiusList[i]));
            wheelList[i].position.copy(wheelPos);
        }
        //orient the wheel models like the chassis and upright
        wheelList[i].quaternion.copy(chassisRB.rotation())
        
        //steering of the front wheels
        if (i === 0 || i === 1){
            let tempRot= maxWheelTurn*WheelTurn;
            wheelList[i].rotateOnWorldAxis(new THREE.Vector3().copy(worldDownDir), tempRot)
        }
        //update the spinning of the wheels
        wheelList[i].rotateZ(WheelSpinRad)
    }

    //printing stuff
    //console.log("chassis rotation", chasRot)

    setTimeout(gameLoop, 10);
};
