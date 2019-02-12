/*
 * Global variables
 */
var meshResolution;

// Particle states
var mass;
var vertexPosition, vertexNormal;
var vertexVelocity;

// Spring properties
var K, restLength; 

// Force parameters
var Cd;
var uf, Cv;

// Extra parameters
var forceVec;
var forceMatrix;


/*
 * Getters and setters
 */
function getPosition(i, j) {
    var id = i*meshResolution + j;
    return vec3.create([vertexPosition[3*id], vertexPosition[3*id + 1], vertexPosition[3*id + 2]]);
}

function setPosition(i, j, x) {
    var id = i*meshResolution + j;
    vertexPosition[3*id] = x[0]; vertexPosition[3*id + 1] = x[1]; vertexPosition[3*id + 2] = x[2];
}

function getNormal(i, j) {
    var id = i*meshResolution + j;
    return vec3.create([vertexNormal[3*id], vertexNormal[3*id + 1], vertexNormal[3*id + 2]]);
}

function getVelocity(i, j) {
    var id = i*meshResolution + j;
    return vec3.create(vertexVelocity[id]);
}

function setVelocity(i, j, v) {
    var id = i*meshResolution + j;
    vertexVelocity[id] = vec3.create(v);
}


/*
 * Provided global functions (you do NOT have to modify them)
 */
function computeNormals() {
    var dx = [1, 1, 0, -1, -1, 0], dy = [0, 1, 1, 0, -1, -1];
    var e1, e2;
    var i, j, k = 0, t;
    for ( i = 0; i < meshResolution; ++i )
        for ( j = 0; j < meshResolution; ++j ) {
            var p0 = getPosition(i, j), norms = [];
            for ( t = 0; t < 6; ++t ) {
                var i1 = i + dy[t], j1 = j + dx[t];
                var i2 = i + dy[(t + 1) % 6], j2 = j + dx[(t + 1) % 6];
                if ( i1 >= 0 && i1 < meshResolution && j1 >= 0 && j1 < meshResolution &&
                     i2 >= 0 && i2 < meshResolution && j2 >= 0 && j2 < meshResolution ) {
                    e1 = vec3.subtract(getPosition(i1, j1), p0);
                    e2 = vec3.subtract(getPosition(i2, j2), p0);
                    norms.push(vec3.normalize(vec3.cross(e1, e2)));
                }
            }
            e1 = vec3.create();
            for ( t = 0; t < norms.length; ++t ) vec3.add(e1, norms[t]);
            vec3.normalize(e1);
            vertexNormal[3*k] = e1[0];
            vertexNormal[3*k + 1] = e1[1];
            vertexNormal[3*k + 2] = e1[2];
            ++k;
        }
}


var clothIndex, clothWireIndex;
function initMesh() {
    var i, j, k;
	
	forceMatrix = new Array(meshResolution*meshResolution*3);
	
	
    vertexPosition = new Array(meshResolution*meshResolution*3);
    vertexNormal = new Array(meshResolution*meshResolution*3);
    clothIndex = new Array((meshResolution - 1)*(meshResolution - 1)*6);
    clothWireIndex = [];

    vertexVelocity = new Array(meshResolution*meshResolution);
    restLength[0] = 4.0/(meshResolution - 1);
    restLength[1] = Math.sqrt(2.0)*4.0/(meshResolution - 1);
    restLength[2] = 2.0*restLength[0];

    for ( i = 0; i < meshResolution; ++i )
        for ( j = 0; j < meshResolution; ++j ) {
            setPosition(i, j, [-2.0 + 4.0*j/(meshResolution - 1), -2.0 + 4.0*i/(meshResolution - 1), 0.0]);
            setVelocity(i, j, vec3.create());

            if ( j < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, i*meshResolution + j + 1);
            if ( i < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, (i + 1)*meshResolution + j);
            if ( i < meshResolution - 1 && j < meshResolution - 1 )
                clothWireIndex.push(i*meshResolution + j, (i + 1)*meshResolution + j + 1);
        }
    computeNormals();

    k = 0;
    for ( i = 0; i < meshResolution - 1; ++i )
        for ( j = 0; j < meshResolution - 1; ++j ) {
            clothIndex[6*k] = i*meshResolution + j;
            clothIndex[6*k + 1] = i*meshResolution + j + 1;
            clothIndex[6*k + 2] = (i + 1)*meshResolution + j + 1;
            clothIndex[6*k + 3] = i*meshResolution + j;
            clothIndex[6*k + 4] = (i + 1)*meshResolution + j + 1;            
            clothIndex[6*k + 5] = (i + 1)*meshResolution + j;
            ++k;
        }
}


/*
 * KEY function: simulate one time-step using Euler's method
 */
function simulate(stepSize) {
	var id;
	
	for (i = 0; i < meshResolution; ++i) {
		for (j = 0; j < meshResolution; ++j) {
			forceVec = calculateForce(i,j);
			id = (i*meshResolution) + j;
			forceMatrix[3*id] = forceVec[0];
			forceMatrix[(3*id) + 1] = forceVec[1];
			forceMatrix[(3*id) + 2] = forceVec[2];
		}
	}
	for (i = (meshResolution - 1); i >= 0; --i) {
		for (j = 0; j < meshResolution; ++j) {
			id = (i*meshResolution) + j;
			var tempVec, tempVec2;
			tempVec	= vec3.scale(vec3.create([forceMatrix[3*id], forceMatrix[3*id + 1], forceMatrix[3*id + 2]]), (1.0 / mass));
			vec3.scale(tempVec, stepSize);
			vec3.add(tempVec, getVelocity(i, j));
			setVelocity(i, j, tempVec);
			if (i == (meshResolution - 1)) {
				if ((j != 0) && (j != (meshResolution - 1))) {
					tempVec2 = vec3.scale(getVelocity(i, j), stepSize);
					vec3.add(tempVec2, getPosition(i, j));
					setPosition(i, j, tempVec2);
				} else {
					setPosition(i, j, getPosition(i, j));
				}
			} else {
				tempVec2 = vec3.scale(getVelocity(i, j), stepSize);
				vec3.add(tempVec2, getPosition(i, j));
				setPosition(i, j, tempVec2);
			}
		}
	}
}

function calculateForce(i, j) {
	var force = vec3.create([0, (-1)*mass*9.8, 0]);	//initializing total force with the force of gravity, MIGHT NEED TO CHANGE TO [0, (-1)*mass*9.8, 0];
	var damp, viscous, tempVec;
	var viscCoeff;
	damp = vec3.scale(getVelocity(i, j), Cd);
	tempVec = vec3.subtract(vec3.create(uf), getVelocity(i,j));
	viscCoeff = Cv * vec3.dot(getNormal(i,j), tempVec);
	viscous = vec3.scale(getNormal(i,j), viscCoeff);	//calculating viscous force
	vec3.subtract(force, damp);		//adding in damping force
	
	/////////////Beginning calculations for the various spring forces. May need to reverse calculations if top row is not 0, but instead is n-1
	if (i < (meshResolution - 1)) {
		vec3.add(force, partialForce(i, j, (i + 1), j, 0));		//call a structural spring going up
		if (j < (meshResolution - 1)) {
			vec3.add(force, partialForce(i, j, (i + 1), (j + 1), 1));	//call a shear spring going UR diagonal
		}
		if (j > 0) {
			vec3.add(force, partialForce(i, j, (i + 1), (j - 1), 1));	//call a shear spring going UL diagonal
		}
		if (i < (meshResolution - 2)) {
			vec3.add(force, partialForce(i, j, (i + 2), j, 2));	//call a flexion spring going up
		}
	}
	if (i > 0) {
		vec3.add(force, partialForce(i, j, (i - 1), j, 0));			//call a structural spring going down
		if (j < (meshResolution - 1)) {
			vec3.add(force, partialForce(i, j, (i - 1), (j + 1), 1));	//call a shear spring going LR diagonal
		}
		if (j > 0) {
			vec3.add(force, partialForce(i, j, (i - 1), (j - 1), 1));	//call a shear spring going LL diagonal
		}
		if (i > 1) {
			vec3.add(force, partialForce(i, j, (i - 2), j, 2));	//call a flexion spring going down
		}
	}
	if (j < (meshResolution - 1)) {
		vec3.add(force, partialForce(i, j, i, (j + 1), 0));		//call a structural spring going right
		if (j < (meshResolution - 2)) {
			vec3.add(force, partialForce(i, j, i, (j + 2), 2));	//call a flexion spring going right
		}
	}
	if (j > 0) {
		vec3.add(force, partialForce(i, j, i, (j - 1), 0));		//call a structural spring going left
		if (j > 1) {
			vec3.add(force, partialForce(i, j, i, (j - 2), 2));	//call a flexion spring going left
		}
	}
	vec3.add(force, viscous); //adding in viscous force
	return force;
}

function partialForce(i1, j1, i2, j2, type) {
	var partForce = vec3.create(vec3.subtract(getPosition(i1, j1), getPosition(i2, j2)));
	var partLength = vec3.length(partForce);
	vec3.scale(partForce, (K[type] * (restLength[type] - partLength)));
	vec3.scale(partForce, (1.0 / partLength));
	return partForce;
}