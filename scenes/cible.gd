extends RigidBody

var d=20
var vitesse=30
var t=0


func _process(delta):
	var teta=atan2(get_translation().x,get_translation().z)
	var w=vitesse/d
	t+=delta
	set_linear_velocity(Vector3(-d*w*sin(w*t),0,d*w*cos(w*t)))



func _on_Area_body_entered(body):
	if body.is_in_group("projectile"):
		body.queue_free()
