import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { Ticker } from './Ticker.js';
import RAPIER from '@dimforge/rapier3d';

var dt = 1 / 60;

var constraintDown = false;
var camera, scene, renderer, gplane=false, clickMarker=false;
var geometry, material, mesh;
var controls,time = Date.now();

var jointBody, constrainedBody, mouseConstraint;

let cubeSizeX= 3;
let cubeSizeY= 0.8;
let cubeSizeZ= 1.8;

// parameters of the car
let chassisWidth= "temporary, the actual value is set in loading of chassis model";
let wheelBase= "temporary, the actual value is set in loading of chassis model";
let hoverDist= 2.1;
let wheelRadius= "not used";
let strength= 2.8;
let damping= 0.45;
let chassisMass= 1;
let maxSpeed= 500;
let maxTurnSpeed= 50;
let accGain= 0.06;
let decGain= 0.05;
let brakeGain= 0.4;
let turnGain= 8;
let maxWheelTurn= Math.PI*0.25;
let grip= 3;
let drag= 0.1;
let wheelSpincoeff= 1;

const ldManager = new THREE.LoadingManager()
const loader = new GLTFLoader(ldManager);
ldManager.onLoad = function() { console.log( 'Loading complete!'); runRapier() }

// To be synced
var meshes=[], bodies=[];
var heartRawPoints, groundRawPoints, groundIndices, chassisRawPoints, chassisBBoxSize;
var globalScale= 0.05;
let wheelA, wheelB, wheelC, wheelD;
let wheelList=[wheelA, wheelB, wheelC, wheelD];
let wheelPathList= ['../assets/wheel.glb'];
let wheelRadiusList= [];


init();
animate();

function init() {
    //create a div to show the 3d scene and insert it between top and bottom bar
    let gameDiv = document.createElement('div');
    gameDiv.classList.add('game');
    document.body.appendChild(gameDiv)


    // scene b
    scene = new THREE.Scene();
    scene.fog = new THREE.Fog( 0x000000, 500, 10000 );

    // camera
    camera = new THREE.PerspectiveCamera( 35, window.innerWidth / (window.innerHeight) , 0.5, 10000 );
    camera.position.set(0,0,0);
    camera.quaternion.setFromAxisAngle(new THREE.Vector3(0,1,0), Math.PI/2);
    scene.add(camera);

    // lights
    var light, materials;
    scene.add( new THREE.AmbientLight( 0x666666 ) );

    light = new THREE.DirectionalLight( 0xffffff, 12 );
    var d = 70;

    light.position.set( -d, d, 0.6*d );

    light.castShadow = true;
    //shadow projection settings
    light.shadow.mapSize.width = 3000;
    light.shadow.mapSize.height = 3000;
    light.shadow.camera.near = d;
    light.shadow.camera.far = 2*d; 
    light.shadow.camera.left = -d;
    light.shadow.camera.right = d;
    light.shadow.camera.top = d;
    light.shadow.camera.bottom = -d;

    scene.add( light );


    // cube
    var cubeGeo = new THREE.BoxGeometry( cubeSizeX, cubeSizeY, cubeSizeZ);
    var cubeMaterial = new THREE.MeshPhysicalMaterial( { color: 0x888888 } );
    let cubeMesh = new THREE.Mesh(cubeGeo, cubeMaterial);
    cubeMesh.castShadow = true;
    cubeMesh.receiveShadow = true;
    meshes.push(cubeMesh);
    scene.add(cubeMesh);

    //create placeholder wheel models
    /*
    for (let j=0; j<wheelList.length; j++) {
        wheelList[j]= (new THREE.Mesh(new THREE.CylinderGeometry(wheelRadius, wheelRadius, 0.2, 6), cubeMaterial));
        wheelList[j].castShadow= true;
        wheelList[j].receiveShadow= true;
        scene.add(wheelList[j]);
    }*/

    //import the wheel models
    for (let k=0; k!==4; k++){
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
    loader.load( '../assets/chassis.glb', function ( gltf ) {

        let chassisMesh= gltf.scene;
        let chassisGeo= makeMergedGeo(chassisMesh);
        chassisRawPoints= chassisGeo.attributes.position.array;
        chassisMesh.castShadow = true;
        chassisMesh.receiveShadow = true;
        chassisMesh.material = cubeMaterial;
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
    loader.load( '../assets/ground.glb', function ( gltf ) {

        let groundGeo= gltf.scene
        // Merge geometries from model meshes (code copied from doppl3r KCC example - Trimesh Class)
        var geometry;
        var geometries = [];
        groundGeo.traverse(function(child) {
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
        
        groundRawPoints = geometry.attributes.position.array
        groundIndices = new Uint32Array(geometry.getIndex().array)
        groundGeo.castShadow = true;
        groundGeo.receiveShadow = true;
        //groundGeo.material= new THREE.MeshStandardMaterial()
        scene.add( groundGeo );

        }, undefined, function ( error ) {
    
            console.log( error );
    
        }
    ); 


    //rendering stuff
    renderer = new THREE.WebGLRenderer( { antialias: true } );
    renderer.setSize( window.innerWidth, window.innerHeight);
    renderer.setClearColor( scene.fog.color );
    renderer.shadowMap.enabled = true;

    gameDiv.appendChild( renderer.domElement );

    renderer.gammaInput = true;
    renderer.gammaOutput = true;
    //renderer.shadowMapEnabled = true;

    window.addEventListener( 'resize', onWindowResize, false );
    
    // orbit controls
    controls = new OrbitControls( camera, renderer.domElement );
    //controls.addEventListener( 'change', render ); // call this only in static scenes (i.e., if there is no animation loop)
    //controls.screenSpacePanning = true;
    //controls.minDistance = 10;
    controls.maxDistance = 35;
    controls.maxPolarAngle = Math.PI / 2;
    controls.target.copy( new THREE.Vector3(425, -40, -120))
    //controls.enableDamping = true ;
    

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
    //controls.handleResize();
    renderer.setSize( window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame( animate );
    render();
    //updates the target of the camera
    controls.update()
}


function render() {
    renderer.render(scene, camera);
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
function runRapier() {

    import('@dimforge/rapier3d').then(RAPIER => {

        let world = new RAPIER.World({ x: 0.0, y: -9.81, z: 0.0 });
        world.timestep = dt
        //console.log("world timestep", world.timestep)

        // Create the Cube physics
        let CubeBodyDesc = RAPIER.RigidBodyDesc.dynamic();
        var CubeBody = world.createRigidBody(CubeBodyDesc)
        CubeBody.setTranslation(new RAPIER.Vector3(-10, 50, -10))
        let CubeColDesc = RAPIER.ColliderDesc.cuboid(cubeSizeX*0.5,cubeSizeY*0.5,cubeSizeZ*0.5);
        let CubeCol = world.createCollider(CubeColDesc, CubeBody)
        bodies.push(CubeBody);


        //create vehicle chassis physics
        let chassisBodyDesc= RAPIER.RigidBodyDesc.dynamic();
        let chassisBody= world.createRigidBody(chassisBodyDesc);
        let chassisColDesc= RAPIER.ColliderDesc.convexHull(chassisRawPoints);
        let chassisCol= world.createCollider(chassisColDesc, chassisBody);
        chassisBody.setTranslation( new RAPIER.Vector3(10, 30, 10))
        bodies.push(chassisBody) 

        console.log("bodies", bodies)
        console.log("meshes", meshes)


        //create ground physics
        let groundBodyDesc = RAPIER.RigidBodyDesc.fixed()
        var groundBody = world.createRigidBody(groundBodyDesc)
        let groundColDesc = RAPIER.ColliderDesc.trimesh(groundRawPoints, groundIndices, 'FIX_INTERNAL_EDGES');
        let groundCol = world.createCollider(groundColDesc, groundBody)

        //initialize drive
        let GoingForward  = false
        let GoingBackward = false
        let GoingLeft     = false
        let GoingRight    = false
        let Braking       = false
        let Flipping      = false
        let WheelTurn     = 0.0
        let WheelSpin     = 0.0
        
        //initialize hover
        let chassisRB = chassisBody;
        let isHoveringList = [false, false, false, false];
        let hoverDistList = [0.0, 0.0, 0.0, 0.0];
        let com= chassisRB.localCom(); //chassis get the center of mass
        let ray = new RAPIER.Ray();
        //calculate the local position of each wheel point where we will cast the ray from
        let localUp = {x:0, y:1, z:0};
        let localDown = {x:0, y:-1, z:0};
        let localFwd={x:1, y:0, z:0};
        let localRight={x:0, y:0, z:1};
        let posA= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5*-1), vector3ByScalar(localFwd, wheelBase*0.5)));
        let posB= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5), vector3ByScalar(localFwd, wheelBase*0.5)));
        let posC= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5*-1), vector3ByScalar(localFwd, wheelBase*0.5*-1)));
        let posD= AddTwoVec3(com, AddTwoVec3(vector3ByScalar(localRight, chassisWidth*0.5), vector3ByScalar(localFwd, wheelBase*0.5*-1)));
        //creer un array de positions locales
        let wheelPointList = [posA, posB, posC, posD];

        //animate the models
        let gameLoop = () => {
            world.step();
            
            //updates display of threejs meshes
            for(var i=0; i !== meshes.length; i++){
                meshes[i].position.copy(bodies[i].translation());
                meshes[i].quaternion.copy(bodies[i].rotation());
            }

            // Get and print the rigid-bodies' position
            //console.log("Cube Body position: ", CubeBody.translation());
            //console.log("Cube Body rotation: ", CubeBody.rotation());
            //console.log("Chassis Body rotation: ", chassisRB.rotation())
            
            //GET KEYS
            //two listeners to get key events
            document.addEventListener("keydown", event => {
                
                if(event.key === "ArrowRight") {
                    GoingRight = true;
                }
                if(event.key === "ArrowLeft") {
                    GoingLeft = true;
                }
                if(event.key === "ArrowUp") {
                    GoingForward = true;
                }
                if(event.key === "ArrowDown") {
                    GoingBackward = true;
                }
                if(event.key === "f") {
                    Flipping = true;
                }
                if(event.key === " ") {
                    Braking = true;
                }
            });
            document.addEventListener("keyup", event => {
                if(event.key === "ArrowRight") {
                    GoingRight = false;
                }
                if(event.key === "ArrowLeft") {
                    GoingLeft = false;
                }
                if(event.key === "ArrowUp") {
                    GoingForward = false;
                }
                if(event.key === "ArrowDown") {
                    GoingBackward = false;
                }
                if(event.key === "r") {
                    Flipping = false;
                }
                if(event.key === " ") {
                    Braking = false;   
                }
            });

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
            //If going forward we want to reach our Max Speed
            if (GoingForward){
                let diff= maxSpeed - currentSpeed;
                //Apply an impulse proportial to the difference between our desired speed and our actual speed
                let imp= vector3ByScalar(worldFwd, (diff * accGain * chassisMass) * powerCoeff);
                chassisRB.applyImpulse(imp)
            }
            //If reversing we reach -Max Speed
            if (GoingBackward){
                let diff= -maxSpeed - currentSpeed;
                let imp= vector3ByScalar(worldFwd, (diff * decGain * chassisMass) * powerCoeff);
                chassisRB.applyImpulse(imp)
            }
            //If braking we reach 0
            if (Braking){
                let imp= vector3ByScalar(worldFwd, (-currentSpeed * brakeGain * chassisMass) * powerCoeff);
                chassisRB.applyImpulse(imp)
            }
            //Check if we can turn. We check if the magnitude of our angular velocity is less than our max turn speed. Max Turning speed depends on velocity
            let mTS= Math.min(Math.abs(currentSpeed), (1-propSpeed) * maxTurnSpeed)
            //console.log("mTS", mTS)
            //console.log("current speed", currentSpeed)
            let canTurn= (Math.abs(dotProduct(angularVel, worldUp)) < mTS)
            //If we can turn and we're turning right
            if (canTurn && GoingLeft){
               //Apply an angular impulse to turn us left
                let imp= vector3ByScalar(worldUp, (turnGain * chassisMass) * powerCoeff);
                if(GoingBackward){ imp= vector3ByScalar(imp, -1)}
                chassisRB.applyTorqueImpulse(imp)
            }
            if (canTurn && GoingRight){
                //Apply an angular impulse to turn us right
                 let imp= vector3ByScalar(worldUp, (-turnGain * chassisMass) * powerCoeff);
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
            //Update the wheels rotations
            if (GoingLeft){
                if(!(WheelTurn < -0.9)){
                    WheelTurn= WheelTurn-0.1;
                }
            }
            if (GoingRight){
                if(!(WheelTurn > 0.9)){
                    WheelTurn= WheelTurn+0.1;
                }
            }
            if (!GoingLeft && !GoingRight){
                if(WheelTurn !== 0){
                    //WheelTurn= WheelTurn*0.8;
                    if(WheelTurn>-0.1 && WheelTurn<0.1){
                        WheelTurn=0;
                    } else if (WheelTurn<0) { 
                        WheelTurn= WheelTurn+0.1;
                    } else if(WheelTurn>0){ 
                        WheelTurn= WheelTurn-0.1
                    }
                }
            }
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
            

            //updates the target of the camera
            controls.target.copy(chassisRB.translation())
            //console.log("orbit controls target", controls.target)

            setTimeout(gameLoop);
        };

        gameLoop();
    });
}