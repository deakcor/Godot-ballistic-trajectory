extends RigidBody

var proj_speed=50
var gravity=9.8/2

func _ready():
	var cible=get_parent().get_node("cible")
	var ballistic=solve_ballistic_arc(get_translation(), proj_speed, cible.get_translation(), cible.get_linear_velocity(), gravity)
	apply_impulse(get_translation(),ballistic[1][0])
#code : https://github.com/forrestthewoods/lib_fts/blob/master/code/fts_ballistic_trajectory.cs
func solve_ballistic_arc(proj_pos, proj_speed, target_pos, target_velocity, gravity):

	var s0 = [Vector3(0,0,0),0]
	var s1 = [Vector3(0,0,0),0]
	
	
	var G = gravity
	
	var A = proj_pos.x
	var B = proj_pos.y
	var C = proj_pos.z
	var M = target_pos.x
	var N = target_pos.y
	var O = target_pos.z
	var P = target_velocity.x
	var Q = target_velocity.y
	var R = target_velocity.z
	var S = proj_speed;
	
	var H = M - A
	var J = O - C
	var K = N - B
	var L = -0.5 * G
	
	# Quartic Coeffecients
	var c0 = L*L;
	var c1 = 2*Q*L;
	var c2 = Q*Q + 2*K*L - S*S + P*P + R*R
	var c3 = 2*K*Q + 2*H*P + 2*J*R
	var c4 = K*K + H*H + J*J
	
	# Solve quartic
	
	var times=[]
	var tmp = SolveQuartic(c0, c1, c2, c3, c4)
	var numTimes=tmp[0]
	for k in range(1,5):
		times.append(tmp[k])
	
	
	#Sort so faster collision is found first
	times.sort()
	
	#Plug quartic solutions into base equations
	#There should never be more than 2 positive, real roots.
	var solutions = [[Vector3(0,0,0),0],[Vector3(0,0,0),0]]
	var numSolutions = 0
	var i=0
	while i<numTimes and numSolutions<2:
		var t=times[i]
		if t>0:
			solutions[numSolutions][0].x = float((H+P*t)/t)
			solutions[numSolutions][0].y = float((K+Q*t-L*t*t)/ t)
			solutions[numSolutions][0].z = float((J+R*t)/t)
			solutions[numSolutions][1]=t
			numSolutions+=1
		i+=1
	
	#Write out solutions
	if (numSolutions > 0):
		s0 = solutions[0]
	if (numSolutions > 1):
		s1 = solutions[1]
	return [numSolutions,s0,s1]
	
func SolveQuartic(c0,c1,c2,c3,c4) :
	var s0
	var s1
	var s2
	var s3
	
	var coeffs = [0,0,0,0]
	var z
	var u
	var v
	var sub
	var A
	var B
	var C
	var D
	var sq_A
	var p
	var q
	var r
	var num
	
	#normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0
	A = c1 / c0
	B = c2 / c0
	C = c3 / c0
	D = c4 / c0
	
	#substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0
	sq_A = A * A
	p = - 3.0/8 * sq_A + B
	q = 1.0/8 * sq_A * A - 1.0/2 * A * B + C
	r = - 3.0/256*sq_A*sq_A + 1.0/16*sq_A*B - 1.0/4*A*C + D
	
	if (IsZero(r)):
		#no absolute term: y(y^3 + py + q) = 0
		
		coeffs[ 3 ] = q
		coeffs[ 2 ] = p
		coeffs[ 1 ] = 0
		coeffs[ 0 ] = 1
		
		var tmp = SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3])
		num= tmp[0]
		s0=tmp[1]
		s1=tmp[2]
		s2=tmp[3]
	else:
		#solve the resolvent cubic ... 
		coeffs[ 3 ] = 1.0/2 * r * p - 1.0/8 * q * q
		coeffs[ 2 ] = - r
		coeffs[ 1 ] = - 1.0/2 * p
		coeffs[ 0 ] = 1
		print(coeffs[0], coeffs[1], coeffs[2], coeffs[3])
		var tmp=SolveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3])
		s0=tmp[1]
		s1=tmp[2]
		s2=tmp[3]
		# ... and take the one real solution ... 
		z = s0;
		
		# ... to build two quadric equations 
		u = z * z - r
		v = 2 * z - p
		if (IsZero(u)):
		    u = 0
		elif (u > 0):
		    u = sqrt(u);
		else:
		    return [0,s0,s1,s2,s3]
		
		if (IsZero(v)):
		    v = 0
		elif(v > 0):
		    v = sqrt(v)
		else:
		    return [0,s0,s1,s2,s3]
		
		coeffs[ 2 ] = z - u
		coeffs[ 1 ] =   -v if q < 0 else v
		coeffs[ 0 ] = 1
		
		tmp = SolveQuadric(coeffs[0], coeffs[1], coeffs[2]);
		num=tmp[0]
		s0=tmp[1]
		s1=tmp[2]
		
		coeffs[ 2 ]= z + u;
		coeffs[ 1 ] = v if q < 0 else -v;
		coeffs[ 0 ] = 1;
		
		if (num == 0):
			tmp = SolveQuadric(coeffs[0], coeffs[1], coeffs[2]);
			num += tmp[0]
			s0=tmp[1]
			s1=tmp[2]
		if (num == 1):
			tmp= SolveQuadric(coeffs[0], coeffs[1], coeffs[2]);
			num += tmp[0]
			s1=tmp[1]
			s2=tmp[2]
		if (num == 2):
			tmp= SolveQuadric(coeffs[0], coeffs[1], coeffs[2]);
			num += tmp[0]
			s2=tmp[1]
			s3=tmp[2]
	
	# resubstitute 
	sub = 1.0/4 * A;
	
	if (num > 0):    s0 -= sub;
	if (num > 1):    s1 -= sub;
	if (num > 2):    s2 -= sub;
	if (num > 3):    s3 -= sub;
	
	return [num,s0,s1,s2,s3];
		
		
func IsZero(d) :
	var eps = 1e-9;
	return d > -eps and d < eps;
func SolveCubic( c0,  c1,  c2,  c3):
	var s0
	var s1
	var s2
	
	var num;
	var sub;
	var A
	var B
	var C;
	var sq_A
	var p
	var q;
	var cb_p
	var D;
	
	#normal form: x^3 + Ax^2 + Bx + C = 0
	A = c1 / c0;
	B = c2 / c0;
	C = c3 / c0;
	
	#  substitute x = y - A/3 to eliminate quadric term:  x^3 +px + q = 0 
	sq_A = A * A;
	p = 1.0/3 * (- 1.0/3 * sq_A + B);
	q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);
	
	#use Cardano's formula
	cb_p = p * p * p;
	D = q * q + cb_p;
	
	if (IsZero(D)) :
		if (IsZero(q)) :
			s0 = 0;
			num = 1;
		else:
			var u = pow(-q, 1.0/3.0);
			s0 = 2 * u;
			s1 = - u;
			num = 2;
	elif(D < 0):
		var phi = 1.0/3 * acos(-q / sqrt(-cb_p));
		var t = 2 * sqrt(-p);
		
		s0 =   t * cos(phi);
		s1 = - t * cos(phi + PI / 3);
		s2 = - t * cos(phi - PI / 3);
		num = 3;
	else:
		var sqrt_D = sqrt(D);
		var u = pow(sqrt_D - q, 1.0/3.0);
		var v = - pow(sqrt_D + q, 1.0/3.0);
		
		s0 = u + v;
		num = 1;
	
	# resubstitute 
	sub = 1.0/3 * A;
	
	if (num > 0):    
		s0 -= sub;
	if (num > 1):    
		s1 -= sub;
	if (num > 2):    
		s2 -= sub;
	
	return [num,s0,s1,s2]

func SolveQuadric( c0,  c1,  c2):
	var s0
	var s1
	
	var p
	var q
	var D;
	
	# normal form: x^2 + px + q = 0 */
	p = c1 / (2 * c0);
	q = c2 / c0;
	
	D = p * p - q;
	var res=0
	if (IsZero(D)):
		s0 = -p;
		res= 1;
	elif (D < 0):
		res= 0;
	else :
		var sqrt_D = sqrt(D);
		
		s0 =   sqrt_D - p;
		s1 = -sqrt_D - p;
		res= 2;
	return [res,s0,s1]

